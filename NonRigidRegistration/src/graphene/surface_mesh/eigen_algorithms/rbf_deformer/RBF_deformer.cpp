//== INCLUDES =================================================================


#include "RBF_deformer.h"

#include <Eigen/Dense>

#define USE_POLYNOMIAL 1


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ==========================================================


RBF_deformer::
RBF_deformer(Surface_mesh& mesh,
             const std::vector<Point>& src_points,
             const std::vector<Point>& tar_points)
{
    if (src_points.size() != tar_points.size())
    {
        std::cerr << "[ERROR] src_points.size() != tar_points.size()" << std::endl;
        return;
    }
    if (src_points.size() == 0)
    {
        std::cerr << "[ERROR] src_points.size() == 0" << std::endl;
        return;
    }

    std::vector<Vec3d> target_positions;
    centers_.clear();
    for (unsigned int i = 0; i < src_points.size(); ++i)
    {
        Vec3d src_point;
        src_point[0] = src_points[i][0];
        src_point[1] = src_points[i][1];
        src_point[2] = src_points[i][2];

        Vec3d tar_point;
        tar_point[0] = tar_points[i][0];
        tar_point[1] = tar_points[i][1];
        tar_point[2] = tar_points[i][2];

        centers_.push_back(src_point);
        target_positions.push_back(tar_point);
    }

    const unsigned int n = centers_.size();

    unsigned int i, j;


#if USE_POLYNOMIAL

    // setup matrix
    Eigen::MatrixXd A(n+4, n+4);
    for (i = 0; i < n; ++i)
    {
        for (j = 0; j < n; ++j)
        {
            A(i, j) = kernel(centers_[j], centers_[i]);
        }
    }

    for (i = 0; i < n; ++i)
    {
        double x = centers_[i][0];
        double y = centers_[i][1];
        double z = centers_[i][2];

        j=n; A(i, j) = A(j, i) = 1;
        ++j; A(i, j) = A(j, i) = x;
        ++j; A(i, j) = A(j, i) = y;
        ++j; A(i, j) = A(j, i) = z;
    }

    for (i = n; i < n+4; ++i)
        for (j = n; j < n+4; ++j)
            A(i, j) = 0.0;

    // setup right hand side
    Eigen::MatrixXd B(n+4, 3);
    for (j = 0; j < n; ++j)
    {
        B(j, 0) = target_positions[j][0];
        B(j, 1) = target_positions[j][1];
        B(j, 2) = target_positions[j][2];
    }

    for (i = n; i < n+4; ++i)
    {
        B(i, 0) = 0.0;
        B(i, 1) = 0.0;
        B(i, 2) = 0.0;
    }

#else

    // setup matrix
    Eigen::MatrixXd A(n, n);
    for (i = 0; i < n; ++i)
    {
        for (j = 0; j < n; ++j)
        {
            A(i, j) = kernel(centers_[j], centers_[i]);
        }
    }

    // setup right hand side
    Eigen::MatrixXd B(n, 3);
    for (j = 0; j < n; ++j)
    {
        B(j, 0) = target_positions[j][0];
        B(j, 1) = target_positions[j][1];
        B(j, 2) = target_positions[j][2];
    }

#endif


    // solve for RBF weights
    Eigen::FullPivLU< Eigen::MatrixXd >  solver;
    // Eigen::HouseholderQR< Eigen::MatrixXd >  solver;
    solver.compute(A);
    Eigen::MatrixXd X = solver.solve(B);

    // copy solution
    weights_.clear();;
    for (j = 0; j < X.rows(); ++j)
    {
        Vec3d Xi;
        Xi[0] = X(j, 0);
        Xi[1] = X(j, 1);
        Xi[2] = X(j, 2);

        weights_.push_back(Xi);
    }

    // deform mesh
    auto mesh_points = mesh.vertex_property<Point>("v:point");
    for (auto v : mesh.vertices())
    {
        // compute deformation
        Point p = mesh_points[v];
        Point new_point = evaluate(p);
        mesh_points[v] = new_point;
    }

    mesh.update_face_normals();
    mesh.update_vertex_normals();
}


//-----------------------------------------------------------------------------


Vec3d
RBF_deformer::
evaluate(const Vec3d& p) const
{
    auto c_it(centers_.begin()), c_end(centers_.end());
    auto w_it(weights_.begin());

    Vec3d f(0,0,0);

    // accumulate RBF basis functions
    for (; c_it!=c_end; ++c_it, ++w_it)
        f += *w_it * kernel(*c_it, p);


#if USE_POLYNOMIAL

    // add constant polynomial
    f += *(w_it++);


    // add linear polynomial
    f += *(w_it++) * p[0];
    f += *(w_it++) * p[1];
    f += *(w_it++) * p[2];

#endif

    return f;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
