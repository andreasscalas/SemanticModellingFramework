//== INCLUDES =================================================================


#include "Nonlinear_discrete_shell.h"

#include <graphene/utility/Stop_watch.h>

#include <cfloat>
#include <Eigen/Cholesky>

#ifdef HAVE_CHOLMOD
#include <Eigen/CholmodSupport>
#define SparseSPDSolver CholmodSupernodalLLT
#else
#define SparseSPDSolver SimplicialLDLT
#endif


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


using namespace graphene::geometry;
using graphene::utility::Stop_watch;


//== IMPLEMENTATION ===========================================================


Nonlinear_discrete_shell::
Nonlinear_discrete_shell()
    : current_mesh_(NULL),
      num_examples_(0),
      current_mesh_idx_(-1),
      base_mesh_set_(false),
      weights_computed_(false),
      mode_(nonlin_mesh_ik)
{
}


//-----------------------------------------------------------------------------


Nonlinear_discrete_shell::
~Nonlinear_discrete_shell()
{
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
set_base_mesh(Surface_mesh& base_mesh)
{
    if (base_mesh.n_vertices() == 0)
    {
        std::cerr << "[ERROR] base mesh is empty." << std::endl;
        return false;
    }
    
    base_mesh_ = base_mesh;
    base_mesh_set_ = true;
    compute_length_and_angle(base_mesh_);

    current_mesh_       = &base_mesh;
    potential_position_ = current_mesh_->vertex_property<Point>("v:potential_position");
    if (!update_free_vertices_and_edges())
    {
        std::cerr << "[ERROR] Can't update free vertices and edges." << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
set_stiffness(const double stiffness_length, const double stiffness_angle)
{
    if (!base_mesh_set_)
    {
        std::cerr << "[ERROR] no base mesh set." << std::endl;
        return false;
    }

    if (!compute_weights_of_length_and_angle(stiffness_length, stiffness_angle))
    {
        return false;
    }

    weights_computed_ = true;

    return true;
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
add_example(const std::string filename)
{
    if (!base_mesh_set_ || !weights_computed_)
    {
        std::cerr << "[ERROR] no base mesh set or no weights computed (stiffness set)." << std::endl;
        return false;
    }

    Surface_mesh example_mesh;
    example_mesh.read(filename);

    if ( example_mesh.n_vertices() != base_mesh_.n_vertices() )
    {
        std::cerr << "[ERROR] in 'Nonlinear_discrete_shell::add_example': Wrong #vertices!" << std::endl;
        return false;
    }

    example_meshes_.push_back(example_mesh);
    num_examples_++;

    Surface_mesh& latest_example = example_meshes_[num_examples_-1];
    compute_length_and_angle(latest_example);

    auto base_angle    = base_mesh_.edge_property<double>("e:initial_angle");
    auto example_angle = latest_example.edge_property<double>("e:initial_angle");

    // add adiff
    auto adiff_example = latest_example.edge_property<double>("e:adiff");
    for ( auto e : latest_example.edges() )
    {
        adiff_example[e] = fabs(example_angle[e] - base_angle[e]);
    }

    model_weight_.push_back(0.0);

    return true;
}


//-----------------------------------------------------------------------------


std::string
Nonlinear_discrete_shell::
next()
{
    if (!base_mesh_set_ || !weights_computed_)
    {
        std::cerr << "[ERROR] no base mesh set or no weights computed (stiffness set)." << std::endl;
        return std::string("Error");
    }

    auto points_current        = current_mesh_->vertex_property<Point>("v:point");
    deformed_points_backup_    = current_mesh_->vertex_property<Point>("v:deformed_point");

    if ( current_mesh_idx_ == -1 &&  num_examples_ > 0 )
    {
        // backup deformed points
        for ( auto v : current_mesh_->vertices() )
            deformed_points_backup_[v] = points_current[v];

        for ( auto v : current_mesh_->vertices() )
            points_current[v] = example_meshes_[0].position(v);

        current_mesh_idx_ = 0;

        return std::string("Example 0");
    }
    else if ( current_mesh_idx_ >= 0 )
    {
        if ( current_mesh_idx_ == num_examples_ - 1 )
        {
            for ( auto v : current_mesh_->vertices() )
                points_current[v] = deformed_points_backup_[v];

            current_mesh_idx_ = -1;

            return std::string("Deformed mesh");
        }
        else
        {
            current_mesh_idx_++;

            for ( auto v : current_mesh_->vertices() )
                points_current[v] = example_meshes_[current_mesh_idx_].position(v);

            std::stringstream ss;
            ss << "Example " << current_mesh_idx_;
            return ss.str();
        }
    }
    else
    {
        return std::string("Error");
    }
}


//-----------------------------------------------------------------------------


std::string
Nonlinear_discrete_shell::
back_to_base()
{
    if (!base_mesh_set_ || !weights_computed_)
    {
        std::cerr << "[ERROR] no base mesh set or no weights computed (stiffness set)." << std::endl;
        return std::string("Error");
    }

    for ( auto v : current_mesh_->vertices() )
        current_mesh_->position(v) = base_mesh_.position(v);

    current_mesh_idx_ = -1;

    return std::string("Base mesh");
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
optimize(unsigned int max_iters)
{
    if (!base_mesh_set_ || !weights_computed_)
    {
        std::cerr << "[ERROR] no base mesh set or no weights computed (stiffness set)." << std::endl;
        return false;
    }

    auto points_current = current_mesh_->vertex_property<Point>("v:point");
    auto base_length    = base_mesh_.edge_property<double>("e:initial_length");
    auto base_angle     = base_mesh_.edge_property<double>("e:initial_angle");

    unsigned int   i, j, k, r, current_col;

    double         error_before_line_search;
    double         error_tmp_old;
    double         error_tmp_new;
    // double         error_tmp;

    Surface_mesh::Vertex  v[4];
    int                   c[4];
    bool                  b[4];
    Vec3d                 g[4];

    unsigned int   m = 2*free_edges_.size();
    unsigned int   n = 3*free_vertices_.size();

    unsigned int J_cols;
    if (mode_ != nonlin_interpolation)
    {
        J_cols = n + num_examples_;
    }
    else
    {
        J_cols = n;
    }

    Eigen::VectorXd  F(m);
    Eigen::VectorXd  X(J_cols);

	std::vector<double>		old_weight(num_examples_);

    Eigen::SparseSPDSolver< SpMat > solver_chol;

    Stop_watch timer;
    timer.start();
    for (unsigned int iter = 0; iter < max_iters; ++iter)
    {
        // copy current vertices to potential new ones
        for (auto v : current_mesh_->vertices())
        {
            new_pos(v) = points_current[v];
        }

        // save modelweight for later use
        for(unsigned int q = 0; q < num_examples_; q++)
        {
            old_weight[q] = model_weight_[q];
        }

        // prepare the matrix, i.e. add partial derivatives
        SpMat J(m, J_cols);
        std::vector<Tripl>  J_coeffs;
        r = 0;
        current_col = 0;
        for (i = 0; i < free_edges_.size(); ++i)
        {
            auto eh   = free_edges_[i];

            auto h = current_mesh_->halfedge(eh, 0);
            v[2]   = current_mesh_->to_vertex(h);
            v[0]   = current_mesh_->to_vertex(current_mesh_->next_halfedge(h));
            h      = current_mesh_->opposite_halfedge(h);
            v[1]   = current_mesh_->to_vertex(h);
            v[3]   = current_mesh_->to_vertex(current_mesh_->next_halfedge(h));

            const Point& p0 = new_pos(v[0]);
            const Point& p1 = new_pos(v[1]);
            const Point& p2 = new_pos(v[2]);
            const Point& p3 = new_pos(v[3]);

            // helper to determine where to insert partial derivatives
            for (k = 0; k < 4; ++k)
            {
                c[k] = 3 * idx( v[k] );
                b[k] = ( c[k] >= 0 );
            }

            // angle stuff
            if (!current_mesh_->is_boundary(eh))
            {
                // compute F
                double diff = angle(p0, p1, p2, p3) - target_angle(eh);
                double we   = weight_angle(eh);
                if( fabs(diff) > 3.1415 ) { we = 0.0; }
                F[r]        = -we * diff;

                // gradient of angle for jacobian
                if (mode_ != nonlin_interpolation)
                {
				    for(unsigned int nm = 0; nm < num_examples_; nm++)
                    {
                        auto example_angle = example_meshes_[nm].edge_property<double>("e:initial_angle");

                        J_coeffs.push_back( Tripl(r, n + nm, (example_angle[eh] - base_angle[eh]) * -we) );
                    }
                }
                gradients_angle(p0, p1, p2, p3, g[0], g[1], g[2], g[3]);
                for (k = 0; k < 4; ++k)
                {
                    if (b[k])
                    {
                        for (j = 0; j < 3; ++j)
                        {
                            J_coeffs.push_back( Tripl(r, c[k] + j, we * g[k][j]) );
                        }
                    }
                }
            }
            r++;

            // length stuff

            // compute F
            F[r] = -weight_length(eh) * (length(p1, p2) - target_length(eh));

            if (mode_ != nonlin_interpolation)
            {
                for(unsigned int nm = 0; nm < num_examples_; nm++)
                {
                    auto example_length = example_meshes_[nm].edge_property<double>("e:initial_length");

                    J_coeffs.push_back( Tripl(r, n + nm, (example_length[eh] - base_length[eh]) * -weight_length(eh)) );
                }
            }

            // gradient of length for jacobian
            gradients_length(p1, p2, g[1], g[2]);

            for (k = 1; k < 3; ++k)
            {
                if (b[k])
                {
                    for (j = 0; j < 3; ++j)
                    {
                        J_coeffs.push_back( Tripl(r, c[k] + j, weight_length(eh) * g[k][j]) );
                    }
                }
            }
            r++;
        }

        // build the matrix
        J.setFromTriplets(J_coeffs.begin(), J_coeffs.end());

        // solve system
        // std::cerr << "solve system" << std::endl;
        SpMat JtJ = J.transpose() * J;
        solver_chol.compute(JtJ);
        if (solver_chol.info() != Eigen::Success)
        {
            std::cerr << "[ERROR] Decomposition failed!" << std::endl;
            return false;
        }
        Eigen::MatrixXd JtF = J.transpose() * F;
        X = solver_chol.solve(JtF);
        if (solver_chol.info() != Eigen::Success)
        {
            std::cerr << "[ERROR] Solving failed!" << std::endl;
            return false;
        }
        // std::cerr << "...done!" << std::endl;

        error_before_line_search = error();
        const double min_step = 1e-10;
        const double improvement = 0.999;

        // LINE SEARCH: STEFAN F.'s VERSION

        for (double s = 1.0; s > min_step; s *= 0.5)
        {
            // update free vertices
            for (i = 0; i < free_vertices_.size(); ++i)
            {
                // comp step
                Point p = Point( X[3*i+0], X[3*i+1], X[3*i+2] );
                p *= s;

                // update
                new_pos(free_vertices_[i]) = points_current[ free_vertices_[i] ] + p;
            }
            if (mode_ != nonlin_interpolation)
            {
                for(unsigned int q = 0; q < num_examples_; q++)
                {
                    // comp step
                    double weights_result = X[n + q];
                    weights_result *= s;

                    // update
                    model_weight_[q] = old_weight[q] + weights_result;
                }
            }
            error_tmp_new = error();

            // step further only if the error is sufficiently decreased
            if (error_tmp_new / error_before_line_search < improvement)
            {
                do // try further halving step size
                {
                    error_tmp_old = error_tmp_new;

                    s *= 0.5;

                    // update free vertices
                    for (i = 0; i < free_vertices_.size(); ++i)
                    {
                        // comp step
                        Point p = Point( X[3*i+0], X[3*i+1], X[3*i+2] );
                        p *= s;

                        // update
                        new_pos(free_vertices_[i]) = points_current[ free_vertices_[i] ] + p;
                    }
                    if (mode_ != nonlin_interpolation)
                    {
                        for(unsigned int q = 0; q < num_examples_; q++)
                        {
                            // comp step
                            double weights_result = X[n + q];
                            weights_result *= s;

                            // update
                            model_weight_[q] = old_weight[q] + weights_result;
                        }
                    }
                    error_tmp_new = error();
                }
                while (error_tmp_new < error_tmp_old);

                // undo last step
                s *= 2.0;

                // update free vertices
                for (i = 0; i < free_vertices_.size(); ++i)
                {
                    // comp step
                    Point p = Point( X[3*i+0], X[3*i+1], X[3*i+2] );
                    p *= s;

                    // update
                    new_pos(free_vertices_[i]) = points_current[ free_vertices_[i] ] + p;
                }
                if (mode_ != nonlin_interpolation)
                {
                    for(unsigned int q = 0; q < num_examples_; q++)
                    {
                        // comp step
                        double weights_result = X[n + q];
                        weights_result *= s;

                        // update
                        model_weight_[q] = old_weight[q] + weights_result;
                    }
                }
                break;
            }
        }

        // update vertex positions
        if (error_tmp_new < error_before_line_search)
        {
            // finally update free vertices
            for (unsigned int i = 0; i < free_vertices_.size(); ++i)
            {
                points_current[ free_vertices_[i] ] = new_pos(free_vertices_[i]);
            }
        }
        else
        {
            // no error decrease -> stop updating
            break;
        }






        // LINE SEARCH: JASCHAS VERSION
        // line search
/*
        bool no_decrease = true;
        for (double s = 1.0; s >= min_step; s *= 0.5)
        {
            // update free vertices
            for (i = 0; i < free_vertices_.size(); ++i)
            {
                // comp step
                Point p = Point( X[3*i+0], X[3*i+1], X[3*i+2] );
                p *= s;

                // update
                new_pos(free_vertices_[i]) = points_current[ free_vertices_[i] ] + p;
            }
            if (mode_ != nonlin_interpolation)
            {
                for(unsigned int q = 0; q < num_examples_; q++)
                {
                    // comp step
                    double weights_result = X[n + q];
                    weights_result *= s;

                    // update
                    model_weight_[q] = old_weight[q] + weights_result;
                }
            }
            error_tmp = error();

            if (error_tmp < error_before_line_search)
            {
                no_decrease = false;
                break;
            }
        }
        if (no_decrease)
        {
            // no error decrease -> stop updating
            break;
        }
        else
        {
            // finally update free vertices
            for (unsigned int i = 0; i < free_vertices_.size(); ++i)
            {
                points_current[ free_vertices_[i] ] = new_pos(free_vertices_[i]);
            }
        }
        // break if the improvement is small only
        if (error_tmp/error_before_line_search >= 0.999)
        {
            break;
        }
*/





    }
    timer.stop();
    std::cerr << "[DEBUG] Time: " << timer.elapsed() << std::endl;

    // update normals
    current_mesh_->update_face_normals();
    current_mesh_->update_vertex_normals();

    return true;
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
mesh_ik(unsigned int max_iters)
{
    mode_ = nonlin_mesh_ik;

    return optimize(max_iters);
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
interpolate(std::vector<double> weights)
{
    if (weights.size() == 0)
    {
        std::cerr << "[ERROR] no examples available!" << std::endl;
        return false;
    }

    if (!base_mesh_set_ || !weights_computed_)
    {
        std::cerr << "[ERROR] no base mesh set or no weights computed (stiffness set)." << std::endl;
        return false;
    }

    if (weights.size() != model_weight_.size())
    {
        std::cerr << "[ERROR] invalid number of weights!" << std::endl;
        return false;
    }

    mode_ = nonlin_interpolation;

    // set current mesh to original base mesh
    for (auto v : current_mesh_->vertices())
    {
        current_mesh_->position(v) = base_mesh_.position(v);
    }

    // update interpolation weights
    for (unsigned int i = 0; i < weights.size(); ++i)
    {
        model_weight_[i] = weights[i];
    }

    return optimize();
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
deformation_transfer(std::string filename_src_undeformed, std::string filename_src_deformed)
{
    Surface_mesh src_undeformed;
    src_undeformed.read(filename_src_undeformed);

    Surface_mesh src_deformed;
    src_deformed.read(filename_src_deformed);


    return deformation_transfer(src_undeformed, src_deformed);
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
deformation_transfer(Surface_mesh& src_undeformed, Surface_mesh& src_deformed)
{
    if (!base_mesh_set_ || !weights_computed_)
    {
        std::cerr << "[ERROR] no base mesh set or no weights computed (stiffness set)." << std::endl;
        return false;
    }

    // clear examples
    model_weight_.clear();
    num_examples_ = 0;
    example_meshes_.clear();
    current_mesh_idx_ = -1;

    mode_ = nonlin_deformation_transfer;

    // set current mesh to original base mesh
    for (auto v : current_mesh_->vertices())
    {
        current_mesh_->position(v) = base_mesh_.position(v);
    }


    if ( src_undeformed.n_vertices() != base_mesh_.n_vertices() )
    {
        std::cerr << "[ERROR] in 'Nonlinear_discrete_shell::set_base_mesh()': A" << std::endl;
        return false;
    }
    if ( src_deformed.n_vertices() != base_mesh_.n_vertices() )
    {
        std::cerr << "[ERROR] in 'Nonlinear_discrete_shell::set_base_mesh()': B" << std::endl;
        return false;
    }

    compute_length_and_angle(src_undeformed);
    compute_length_and_angle(src_deformed);

    auto base_length = base_mesh_.edge_property<double>("e:initial_length");
    auto base_angle  = base_mesh_.edge_property<double>("e:initial_angle");

    auto spec_target_length = base_mesh_.edge_property<double>("e:spec_target_length");
    auto spec_target_angle  = base_mesh_.edge_property<double>("e:spec_target_angle");

    auto src_undeformed_length = src_undeformed.edge_property<double>("e:initial_length");
    auto src_undeformed_angle  = src_undeformed.edge_property<double>("e:initial_angle");

    auto src_deformed_length = src_deformed.edge_property<double>("e:initial_length");
    auto src_deformed_angle  = src_deformed.edge_property<double>("e:initial_angle");

    for ( auto e : base_mesh_.edges() )
    {
        const double factor_length = src_deformed_length[e] / src_undeformed_length[e];
        spec_target_length[e] = base_length[e] * factor_length;

        const double diff_angle = src_deformed_angle[e] - src_undeformed_angle[e];
        spec_target_angle[e]  = base_angle[e] + diff_angle;
    }

    return optimize();
}


//-----------------------------------------------------------------------------


void
Nonlinear_discrete_shell::
compute_length_and_angle(Surface_mesh& mesh)
{
    auto points         = mesh.vertex_property<Point>("v:point");
    auto initial_length = mesh.edge_property<double>("e:initial_length");
    auto initial_angle  = mesh.edge_property<double>("e:initial_angle");

    for (auto e : mesh.edges())
    {
        auto h  = mesh.halfedge(e, 0);
        auto v2 = mesh.to_vertex(h);
        auto v0 = mesh.to_vertex(mesh.next_halfedge(h));
        h  = mesh.opposite_halfedge(h);
        auto v1 = mesh.to_vertex(h);
        auto v3 = mesh.to_vertex(mesh.next_halfedge(h));

        const Point& p0 = points[v0];
        const Point& p1 = points[v1];
        const Point& p2 = points[v2];
        const Point& p3 = points[v3];

        initial_length[e] = length(p1, p2);

        if (!mesh.is_boundary(e))
        {
            initial_angle[e] = angle(p0, p1, p2, p3);
        }
        else
        {
            initial_angle[e] = 0.0;
        }
    }
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
compute_weights_of_length_and_angle(const double stiffness_length, const double stiffness_angle)
{
    if (!base_mesh_set_)
    {
        std::cerr << "[ERROR] no base mesh set." << std::endl;
        return false;
    }

    auto points         = base_mesh_.vertex_property<Point>("v:point");
    weight_edge_length_ = base_mesh_.edge_property<double>("e:weight_length");
    weight_edge_angle_  = base_mesh_.edge_property<double>("e:weight_angle");

    // find global scaling
    Point bb_min(FLT_MAX, FLT_MAX, FLT_MAX);
    Point bb_max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for ( auto v : base_mesh_.vertices() )
    {
        bb_min.minimize( points[v] );
        bb_max.maximize( points[v] );
    }
    double bb_size = norm(bb_max - bb_min);

    for (auto e : base_mesh_.edges())
    {
        auto h  = base_mesh_.halfedge(e, 0);
        auto v2 = base_mesh_.to_vertex(h);
        auto v0 = base_mesh_.to_vertex(base_mesh_.next_halfedge(h));
        h  = base_mesh_.opposite_halfedge(h);
        auto v1 = base_mesh_.to_vertex(h);
        auto v3 = base_mesh_.to_vertex(base_mesh_.next_halfedge(h));

        const Point& p0 = points[v0];
        const Point& p1 = points[v1];
        const Point& p2 = points[v2];
        const Point& p3 = points[v3];

        double l = length(p1, p2);
        weight_edge_length_[e] = sqrt( stiffness_length / bb_size / l ); // STEFAN VERSION
//        weight_edge_length_[e] = sqrt( stiffness_length / (l * l) ); // PAPER VERSION

        if (!base_mesh_.is_boundary(e))
        {
            weight_edge_angle_[e] = sqrt( 3.0 * stiffness_angle * l * l / (area(p0, p1, p2) + area(p1, p2, p3)) ); // STEFAN VERSION
//            weight_edge_angle_[e] = sqrt( stiffness_angle * l * l / (area(p0, p1, p2) + area(p1, p2, p3)) ); // PAPER VERSION
        }
        else
        {
            weight_edge_angle_[e] = 0.0;
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


const double
Nonlinear_discrete_shell::
length(const Point& p0, const Point& p1) const
{
    return norm(p0 - p1);
}


//-----------------------------------------------------------------------------


const double
Nonlinear_discrete_shell::
area(const Point& p0, const Point& p1, const Point& p2) const
{
    return 0.5 * norm( cross( (p1 - p0) , (p2 - p0) ) );
}


//-----------------------------------------------------------------------------


const double
Nonlinear_discrete_shell::
angle(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const
{
    const Point n1 = (cross((p1-p0) , (p2-p0))).normalize();
    const Point n2 = (cross((p2-p3) , (p1-p3))).normalize();
    const Point  e = (p2-p1).normalize();

    return atan2( dot(cross(n1,n2),e) , dot(n1,n2) );
}


//-----------------------------------------------------------------------------


void
Nonlinear_discrete_shell::
gradients_length(const Point& p0, const Point& p1, Vec3d& g0, Vec3d& g1) const
{
    g0 = (p0 - p1).normalize();

    g1 = - g0;
}


//-----------------------------------------------------------------------------


void
Nonlinear_discrete_shell::
gradients_angle(const Point& p0, const Point& p1, const Point& p2, const Point& p3, Vec3d& g0, Vec3d& g1, Vec3d& g2, Vec3d& g3) const
{
    Point n1 = cross( (p1 - p0) , (p2 - p0) );
    n1 /= -sqrnorm(n1);

    Point n2 = cross( (p2 - p3) , (p1 - p3) );
    n2 /= -sqrnorm(n2);

    Point  e = (p2 - p1);
    double l = norm(e);

    g0 = n1 * l;
    g3 = n2 * l;
    e /= l;
    g1 = dot((p0 - p2), e) * n1 + dot((p3 - p2), e) * n2;
    g2 = dot((p1 - p0), e) * n1 + dot((p1 - p3), e) * n2;
}


//-----------------------------------------------------------------------------


const double
Nonlinear_discrete_shell::
error()
{
    double error(0.0);

    for (unsigned int i = 0; i < free_edges_.size(); ++i)
    {
        auto eh = free_edges_[i];

        auto h  = current_mesh_->halfedge(eh, 0);
        auto v2 = current_mesh_->to_vertex(h);
        auto v0 = current_mesh_->to_vertex(current_mesh_->next_halfedge(h));
        h  = current_mesh_->opposite_halfedge(h);
        auto v1 = current_mesh_->to_vertex(h);
        auto v3 = current_mesh_->to_vertex(current_mesh_->next_halfedge(h));

        const Point& p0 = new_pos(v0);
        const Point& p1 = new_pos(v1);
        const Point& p2 = new_pos(v2);
        const Point& p3 = new_pos(v3);

        // length error
        double diff = length(p1, p2) - target_length(eh);
        diff *= weight_length(eh);
        error += diff * diff;

        // angle error
        if (!current_mesh_->is_boundary(eh))
        {
            diff   = angle(p0, p1, p2, p3) - target_angle(eh);
            diff  *= weight_angle(eh);
            error += diff * diff;
        }
    }

    return error;
}


//-----------------------------------------------------------------------------


bool
Nonlinear_discrete_shell::
update_free_vertices_and_edges()
{
    if (!base_mesh_set_)
    {
        std::cerr << "[ERROR] no base mesh set." << std::endl;
        return false;
    }

    if (!current_mesh_)
    {
        std::cerr << "[ERROR] No mesh available! Exit." << std::endl;
        return false;
    }

    auto region = current_mesh_->get_vertex_property<Region_type>("v:region");
    if (!region)
    {
        std::cerr << "[ERROR] Can't find region type! Exit." << std::endl;
        return false;
    }

    vidx_ = current_mesh_->vertex_property<int>("v:vidx");

    // collect free vertices
    free_vertices_.clear();
    for (auto v : current_mesh_->vertices())
    {
        if (region[v] == deformable)
        {
            idx(v) = free_vertices_.size();
            free_vertices_.push_back(v);
        }
        else
        {
            idx(v) = -1;
        }
    }

    // collect free edges
    free_edges_.clear();
    for (auto e : current_mesh_->edges())
    {
        auto h  = current_mesh_->halfedge(e, 0);
        auto v2 = current_mesh_->to_vertex(h);
        auto v0 = current_mesh_->to_vertex(current_mesh_->next_halfedge(h));
        h  = current_mesh_->opposite_halfedge(h);
        auto v1 = current_mesh_->to_vertex(h);
        auto v3 = current_mesh_->to_vertex(current_mesh_->next_halfedge(h));

        if ( (region[v0] == deformable) || (region[v1] == deformable) || (region[v2] == deformable) || (region[v3] == deformable) )
        {
            free_edges_.push_back(e);
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


double
Nonlinear_discrete_shell::
target_length(const Surface_mesh::Edge eh)
{
    if ((mode_ == nonlin_mesh_ik) ||
        (mode_ == nonlin_interpolation))
    {
        auto base_length = base_mesh_.edge_property<double>("e:initial_length");

        double length = base_length[eh];

        for(int i = 0; i < num_examples_; i++)
        {
            auto example_length = example_meshes_[i].edge_property<double>("e:initial_length");

            length += model_weight_[i] * (example_length[eh] - base_length[eh]);
        }

        return length;
    }
    else if (mode_ == nonlin_deformation_transfer)
    {
        auto spec_target_length = base_mesh_.edge_property<double>("e:spec_target_length");
        return spec_target_length[eh];
    }
}



//-----------------------------------------------------------------------------


double
Nonlinear_discrete_shell::
target_angle(const Surface_mesh::Edge eh)
{
    if ((mode_ == nonlin_mesh_ik) ||
        (mode_ == nonlin_interpolation))
    {
        auto base_angle = base_mesh_.edge_property<double>("e:initial_angle");

        double angle = base_angle[eh];

        for(int i = 0; i < num_examples_; i++)
        {
            auto example_angle = example_meshes_[i].edge_property<double>("e:initial_angle");

            angle += model_weight_[i] * (example_angle[eh] - base_angle[eh]);
        }

        return angle;
    }
    else if (mode_ == nonlin_deformation_transfer)
    {
        auto spec_target_angle = base_mesh_.edge_property<double>("e:spec_target_angle");
        return spec_target_angle[eh];
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
