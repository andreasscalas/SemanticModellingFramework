//=============================================================================
#ifndef GRAPHENE_MYHELPER_H
#define GRAPHENE_MYHELPER_H
//=============================================================================

//== INCLUDES ===================================================================

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cfloat>

#include <graphene/utility/Stop_watch.h>

#ifdef _WIN32

#else
  #include <dirent.h>
#endif


#include <graphene/types.h>
#include <graphene/geometry/Matrix3x3.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/Triangle_kD_tree.h>
#include <graphene/geometry/Matrix4x4.h>
#include <graphene/geometry/bary_coord.h>
#include <graphene/geometry/distance_point_triangle.h>
#include <graphene/geometry/Point_BSP_tree.h>
#include <graphene/geometry/Point_set.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Landmarks_manager.h>
#include <graphene/surface_mesh/algorithms/templatefitting/types/Correspondence.h>
#include <graphene/surface_mesh/algorithms/templatefitting/types/Correspondences_settings.h>

#include <graphene/character/data_structure/Character.h>
#include <graphene/geometry/Dual_quaternion.h>

#include <opencl/CL_state.h>

#include "../settings.h"


//== HELPER ===================================================================
namespace graphene
{
namespace surface_mesh
{

// Creates a list of files (absolute path) from directory 'dir' with extension 'ext'
static std::vector<std::string>
read_filelist_from_dir(std::string dir_string, std::string ext)
{
#ifdef _WIN32
    std::cerr << "Currently not implemented for windows!" << std::endl;
    return std::vector<std::string>();
#else
    std::vector<std::string> fileList;

    std::string extension = ".";
    extension.append(ext);

    DIR * dir;
    struct dirent *dirpointer;

    // Open directory
    if ((dir = opendir(dir_string.c_str())) == NULL) {
        std::cerr << "Error at opendir ..." << std::endl;
        exit(1);
    }

    // Read the complete directory
    while ((dirpointer = readdir(dir)) != NULL) {
        std::string dirStr = dir_string;
        std::string tmpStr = (*dirpointer).d_name;
        if (tmpStr.find(extension) != tmpStr.npos) {
            fileList.push_back(dirStr.append("/").append(tmpStr));
        } else {
            continue;
        }
    }

    // Close pointer for reading
    if (closedir(dir) == -1) {
        std::cerr << "Error at closedir: " << dir_string << std::endl;
    }

    return fileList;
#endif
}


//-----------------------------------------------------------------------------


static void
transform_mesh(graphene::surface_mesh::Surface_mesh& mesh,
               const graphene::Mat4f& M)
{
#ifdef BE_VERBOSE
    std::cerr << "in: 'transform_mesh(...)'" << std::endl;
#endif

    auto template_points = mesh.vertex_property<graphene::Point>("v:point");
    for (auto v : mesh.vertices())
    {
        graphene::Point &p = template_points[v];
        p = affine_transform(M, p);
        //template_points[v] = p;
    }

    //mesh.update_face_normals();
    mesh.update_vertex_normals();
}


//-----------------------------------------------------------------------------


static graphene::Mat4f
build_transformation_from_abc(const double a, const double b, const double c)
{
    graphene::Mat4f result = graphene::Mat4f::identity();

// REAL ROTATION
    result(0,0) = std::cos(c)*std::cos(b);
    result(0,1) = -std::sin(c)*std::cos(a) + std::cos(c)*std::sin(b)*std::sin(a);
    result(0,2) = std::sin(c)*std::sin(a) + std::cos(c)*std::sin(b)*std::cos(a);
    result(1,0) = std::sin(c)*std::cos(b);
    result(1,1) = std::cos(c)*std::cos(a) + std::sin(c)*std::sin(b)*std::sin(a);
    result(1,2) = -std::cos(c)*std::sin(a) + std::sin(c)*std::sin(b)*std::cos(a);
    result(2,0) = -std::sin(b);
    result(2,1) = std::cos(b)*std::sin(a);
    result(2,2) = std::cos(b)*std::cos(a);

// LINEARIZED FORM (BETTER VERSION)
/*
  result(0,0) = 1.0;
  result(0,1) = a*b - c;
  result(0,2) = a*c + b;
  result(1,0) = c;
  result(1,1) = a*b*c + 1.0;
  result(1,2) = b*c - a;
  result(2,0) = -b;
  result(2,1) = a;
  result(2,2) = 1.0;
*/

// LINEARIZED FORM (POOR VERSION)
/*
  result(0,0) = 1.0;
  result(0,1) = -c;
  result(0,2) = b;
  result(1,0) = c;
  result(1,1) = 1.0;
  result(1,2) = -a;
  result(2,0) = -b;
  result(2,1) = a;
  result(2,2) = 1.0;
*/

    const double tmp_determinant = result(0,0)*result(1,1)*result(2,2) + result(0,1)*result(1,2)*result(2,0) + result(0,2)*result(1,0)*result(2,1) - result(2,0)*result(1,1)*result(0,2) - result(2,1)*result(1,2)*result(0,0) - result(2,2)*result(1,0)*result(0,1);
//    std::cerr << "Determinant of R(a,b,c): " << tmp_determinant << std::endl;

    if (tmp_determinant < 0.0)
    {
        std::cerr << "[ERROR] in build_transformation_from_abc()" << std::endl;
        exit(1);
    }

    return result;
}


//-----------------------------------------------------------------------------


static graphene::Mat4f
build_transformation_from_abcxyz(const double a, const double b, const double c, const double tx, const double ty, const double tz)
{
    graphene::Mat4f result = build_transformation_from_abc(a, b, c);

    result(0, 3) = tx;
    result(1, 3) = ty;
    result(2, 3) = tz;

    return result;
}


//-----------------------------------------------------------------------------


static std::string
to_string_with_precision(const double a_value, const int n = 6)
{
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}


//-----------------------------------------------------------------------------


static bool
is_point_boundary(graphene::surface_mesh::Surface_mesh& _mesh,
                  const graphene::Point &_p,
                 const graphene::surface_mesh::Surface_mesh::Face &_face)
{
    // std::cerr << "in: 'is_point_boundary(...)'" << std::endl;

    // is the face at a boundary
    if( !_mesh.is_boundary(_face) )
    {
        return false;
    }

    auto points = _mesh.vertex_property<graphene::Point>("v:point");

    // get triangle vertices
    auto fvit = _mesh.vertices(_face);
    const auto   v0 = *fvit;
    const bool b_v0 = _mesh.is_boundary(v0);
    const auto   v1 = *(++fvit);
    const bool b_v1 = _mesh.is_boundary(v1);
    const auto   v2 = *(++fvit);
    const bool b_v2 = _mesh.is_boundary(v2);

    const graphene::Point p0 = points[v0];
    const graphene::Point p1 = points[v1];
    const graphene::Point p2 = points[v2];

    // get barycentric coordinates
    const graphene::Point b = graphene::geometry::barycentric_coordinates(_p, p0, p1, p2);
    const float epsilon = 0.01;

    // check first location on edge
    if(b_v0&&b_v1)
    {
        if(b[2]<epsilon){return true;}
    }
    if(b_v1&&b_v2)
    {
        if(b[0]<epsilon){return true;}
    }
    if(b_v2&&b_v0)
    {
        if(b[1]<epsilon){return true;}
    }
    // check now location on boundary point
    if(b_v0){if(b[1]<epsilon&&b[2]<epsilon)
        {return true;}}
    if(b_v1){if(b[2]<epsilon&&b[0]<epsilon)
        {return true;}}
    if(b_v2){if(b[0]<epsilon&&b[1]<epsilon)
        {return true;}} 
    return false;
}


//-----------------------------------------------------------------------------


static bool
check_if_outside( const graphene::Point& point, const graphene::surface_mesh::Surface_mesh& mesh )
{
    // kd tree of eye mesh
    const graphene::surface_mesh::Triangle_kD_tree mesh_kd( mesh );

    // find closest point to sample i on template
    const auto nn_tri = mesh_kd.nearest( point );
    const auto face = nn_tri.face;
    const auto fnormals = mesh.get_face_property<graphene::Point>("f:normal");
    const graphene::Normal face_normal = fnormals[face];
    const graphene::Point x_nearest = nn_tri.nearest;

    const double dot = graphene::dot( face_normal, point - x_nearest );

    if ( dot > 0 )
    {
        return true;
    }
    else
    {
        return false;
    }
}


//-----------------------------------------------------------------------------


static bool
compute_correspondences_mesh2ps_CPU(Surface_mesh *template_mesh, geometry::Point_set *point_set, const Correspondences_settings &correspondences_settings, std::vector<Correspondence> &correspondences)
{

    graphene::geometry::Point_BSP_tree point_set_bsp(point_set->points_);

    auto template_points  = template_mesh->vertex_property<Point>("v:point");
    auto template_normals = template_mesh->vertex_property<Normal>("v:normal");

    // do not build Ematch correspondences for specified vertices or triangles with specified vertices
    auto vertex_omit = template_mesh->get_vertex_property<bool>("v:omit");

    auto templ_local_weight = template_mesh->get_vertex_property<double>("v:fitting_weight");

    Triangle_kD_tree template_mesh_kd(*template_mesh, 100, 60);

    // compute correspondences
    double d_max_mesh2ps(FLT_MIN);
    if ( correspondences_settings.w_distance_1ddMax )
    {
        for (auto v : template_mesh->vertices())
        {
            // find closest point to sample i
            const auto nn_ps = point_set_bsp.nearest(template_points[v]);

            d_max_mesh2ps = std::max( d_max_mesh2ps, (double) distance(point_set->points_[nn_ps.nearest], template_points[v]) );
        }
    }
    // double d_min_mesh2ps(FLT_MAX);
    // for (auto v : template_mesh_->vertices())
    // {
    //     // find closest point to sample i
    //     const auto nn_ps = point_set_bsp.nearest(template_points[v]);

    //     d_min_mesh2ps = std::min( d_min_mesh2ps, (double) distance(point_set_->points_[nn_ps.nearest], template_points[v]) );
    // }
    for (auto v : template_mesh->vertices())
    {
        // find closest point to sample i on point-set
        const auto nn_ps = point_set_bsp.nearest(template_points[v]);

        // skip if mesh vertex is specified
        if (vertex_omit)
        {
            if (vertex_omit[v])
            {
                // std::cerr << "skip specified vertex !!!" << std::endl;
                continue;
            }
        }

        if (templ_local_weight)
        {
            if (templ_local_weight[v] < 1e-5)
            {
                // std::cerr << "skip since local weight is zero !!!" << std::endl;
                continue;
            }
        }

        // skip if target is on boundary
        // if ( correspondences_settings.boundary_filter )
        // {
        //     if ( template_mesh_->is_boundary(v) )
        //     {
        //         continue;
        //     }
        // }
        // filter correspondences
        if ( correspondences_settings.filter_normals )
        {
            // check normal vector
            if ( fabs( norm(point_set->normals_[nn_ps.nearest]) - 1.0) > 1e-5 )
            {
                std::cerr << "ERROR: Normal does not have length 1 !!!" << std::endl;
                exit(1);
            }
            if ( fabs( norm(template_normals[v]) - 1.0) > 1e-5 )
            {
                std::cerr << "ERROR: Normal does not have length 1 !!!" << std::endl;
                exit(1);
            }
            if (dot(point_set->normals_[nn_ps.nearest], template_normals[v]) < std::cos(correspondences_settings.normal_until * M_PI / 180.0))
            {
                continue;
            }
        }
        if ( correspondences_settings.filter_dist )
        {
            // check distance
            if ( distance(point_set->points_[nn_ps.nearest], template_points[v]) > correspondences_settings.dist_until )
            {
                continue;
            }
        }
        if ( correspondences_settings.symmetry_filter )
        {
            const auto nn_tri = template_mesh_kd.nearest(point_set->points_[nn_ps.nearest]);
            // check congruency
            if (norm(nn_tri.nearest - template_points[v]) > correspondences_settings.symmetry_epsilon )
            {
                continue;
            }
        }

        // store correspondence
        Correspondence c;
        c.on_template    = template_points[v];
        c.on_template_n  = template_normals[v];  // TODO EVTL. RAUS und mehr
        c.on_template_vh = v;
        //        std::cerr << "c.sample_vh.idx(): " << c.sample_vh.idx() << std::endl;
        c.on_ps          = point_set->points_[nn_ps.nearest];
        c.on_ps_n        = point_set->normals_[nn_ps.nearest];
        c.distance_point2point = distance(point_set->points_[nn_ps.nearest], template_points[v]);
        c.constr_dir     = corresp_dir_mesh2ps;
        // weight
        c.weight        = 1.0;
        if (templ_local_weight)
        {
            c.weight *= templ_local_weight[v];
        }
        if ( correspondences_settings.w_distance_1r )
        {
            double dist = 100.0*std::numeric_limits<double>::epsilon() + distance(point_set->points_[nn_ps.nearest], template_points[v]);
            c.weight *= 1.0 / ( dist );
        }
        if ( correspondences_settings.w_distance_1ddMax )
        {
            c.weight *= ( 1.0 - ( distance(point_set->points_[nn_ps.nearest], template_points[v]) / d_max_mesh2ps ) );
        }
        if ( correspondences_settings.w_dot_nn )
        {
            c.weight *= std::max( 0.0, (double) dot(point_set->normals_[nn_ps.nearest], template_normals[v]) );
        }
        if ( correspondences_settings.w_distance_huber )
        {
            std::cerr << "compute_correspondences_mesh2ps_CPU: huber currently not supported" << std::endl;
            exit(1);

            double dist  = distance(point_set->points_[nn_ps.nearest], template_points[v]);
            double h_thr = correspondences_settings.w_distance_huber_value;
            if ( dist <= h_thr )
            {
                c.weight *= 1.0;
            }
            else
            {
                c.weight *= h_thr / dist;
            }
        }

        correspondences.push_back(c);
    }

    return true;

}


//-----------------------------------------------------------------------------


static bool
compute_correspondences_ps2mesh_CPU(Surface_mesh* template_mesh, geometry::Point_set* point_set,  const Correspondences_settings& correspondences_settings, std::vector<Correspondence>& correspondences)
{
    // std::cerr << "in: 'compute_correspondences(...)'" << std::endl;

    // do not build Ematch correspondences for specified vertices or triangles with specified vertices
    auto vertex_omit = template_mesh->get_vertex_property<bool>("v:omit");

    auto templ_local_weight = template_mesh->get_vertex_property<double>("v:fitting_weight");

    // create kd trees if not existing/outdated and if necessary
    Triangle_kD_tree template_mesh_kd(*template_mesh, 100, 60);

    graphene::geometry::Point_BSP_tree point_set_bsp(point_set->points_);
    point_set_bsp.build(10, 99);

    auto template_points  = template_mesh->vertex_property<Point>("v:point");
    auto template_normals = template_mesh->vertex_property<Normal>("v:normal");

    // compute correspondences
    const unsigned int np = point_set->points_.size();
    //        correspondences.reserve(correspondences.size() + np);
    double d_max_ps2mesh(FLT_MIN);
    if ( correspondences_settings.w_distance_1ddMax )
    {
        for (unsigned int i = 0; i < np; ++i)
        {
            // find closest point to sample i
            const auto nn_tri = template_mesh_kd.nearest(point_set->points_[i]);

            d_max_ps2mesh = std::max( d_max_ps2mesh, (double) distance(nn_tri.nearest, point_set->points_[i]) );
        }
    }
    // double d_min_ps2mesh(FLT_MAX);
    // for (unsigned int i = 0; i < np; ++i)
    // {
    //     // find closest point to sample i
    //     const auto nn_tri = template_mesh_kd.nearest(point_set_->points_[i]);

    //     d_min_ps2mesh = std::min( d_min_ps2mesh, (double) distance(nn_tri.nearest, point_set_->points_[i]) );
    // }
    // double sum_residuals = 0.0;
    // for (unsigned int i = 0; i < np; ++i)
    // {
    //     // find closest point to sample i
    //     const auto nn_tri = template_mesh_kd.nearest(point_set_->points_[i]);

    //     sum_residuals += std::pow( (double) distance(nn_tri.nearest, point_set_->points_[i]), 2);
    // }
    // sum_residuals /= np;
    // sum_residuals = sqrt(sum_residuals);
    for (unsigned int i = 0; i < np; ++i)
    {
        // find closest point to sample i on template
        const auto nn_tri = template_mesh_kd.nearest(point_set->points_[i]);

        // get vertices of closest triangle
        auto fvit = template_mesh->vertices(nn_tri.face);
        const auto v0 = *fvit;
        const auto v1 = *(++fvit);
        const auto v2 = *(++fvit);
        const Point p0 = template_points[v0];
        const Point p1 = template_points[v1];
        const Point p2 = template_points[v2];
        const Normal n0 = template_normals[v0];
        const Normal n1 = template_normals[v1];
        const Normal n2 = template_normals[v2];
        // get barycentric coordinates
        const Point b = graphene::geometry::barycentric_coordinates(nn_tri.nearest, p0, p1, p2);
        Normal n_on_template = b[0]*n0 + b[1]*n1 + b[2]*n2;
        n_on_template.normalize();

        // skip if nearest triangle has a specified vertex
        if (vertex_omit)
        {
            if (vertex_omit[v0] || vertex_omit[v1] || vertex_omit[v2])
            {
                // std::cerr << "skip specified vertex !!!" << std::endl;
                continue;
            }
        }

        if (templ_local_weight)
        {
            if (templ_local_weight[v0] < 1e-5 || templ_local_weight[v1] < 1e-5 || templ_local_weight[v2] < 1e-5)
            {
                // std::cerr << "skip since local weight is zero !!!" << std::endl;
                continue;
            }
        }

        // skip if target is on boundary
        if ( correspondences_settings.boundary_filter )
        {
            if ( is_point_boundary(*template_mesh, nn_tri.nearest, nn_tri.face) )
            {
                continue;
            }
        }

        // filter correspondences
        if ( correspondences_settings.filter_normals )
        {
            // check normal vector
            if ( fabs( norm(point_set->normals_[i]) - 1.0) > 1e-5 )
            {
                std::cerr << "ERROR1: Normal does not have length 1 !!! -> " << norm(point_set->normals_[i]) << std::endl;
                exit(1);
            }
            if ( fabs( norm(n_on_template) - 1.0) > 1e-5 )
            {
                std::cerr << "ERROR2: Normal does not have length 1 !!!" << std::endl;
                exit(1);
            }
            if (dot(n_on_template, point_set->normals_[i]) < std::cos(correspondences_settings.normal_until * M_PI / 180.0))
            {
                continue;
            }
        }
        if ( correspondences_settings.filter_dist )
        {
            // check distance
            if ( distance(nn_tri.nearest, point_set->points_[i] ) > correspondences_settings.dist_until)
            {
                continue;
            }
        }
        if ( correspondences_settings.symmetry_filter )
        {
            const auto nn_ps = point_set_bsp.nearest(nn_tri.nearest);
            // check congruency
            if (norm(point_set->points_[nn_ps.nearest] - point_set->points_[i]) > correspondences_settings.symmetry_epsilon)
            {
                continue;
            }
        }

        // store correspondence
        Correspondence c;
        c.face          = nn_tri.face;
        c.on_ps         = point_set->points_[i];
        c.on_ps_n       = point_set->normals_[i];
        c.on_template   = nn_tri.nearest;
        c.on_template_bc= b;
        c.distance_point2point = distance( nn_tri.nearest, point_set->points_[i] );
        c.constr_dir    = corresp_dir_ps2mesh;
        c.on_template_n = n_on_template;  // TODO EVTL. RAUS und mehr
        // weight
        c.weight        = 1.0;
        if (templ_local_weight)
        {
            const double local_weight = b[0]*templ_local_weight[v0] + b[1]*templ_local_weight[v1] + b[2]*templ_local_weight[v2];
            c.weight *= local_weight;
        }
        if ( correspondences_settings.w_distance_1r )
        {
            double dist = 100.0*std::numeric_limits<double>::epsilon() + distance(nn_tri.nearest, point_set->points_[i]);
            c.weight *= 1.0 / ( dist );
        }
        if ( correspondences_settings.w_distance_1ddMax )
        {
            c.weight *= ( 1.0 - ( distance(nn_tri.nearest, point_set->points_[i]) / d_max_ps2mesh ) );
        }
        if ( correspondences_settings.w_dot_nn )
        {
            c.weight *= std::max( 0.0f, dot(n_on_template, point_set->normals_[i]) );
        }
        if ( correspondences_settings.w_distance_huber )
        {
            std::cerr << "compute_correspondences_ps2mesh_CPU: huber currently not supported" << std::endl;
            exit(1);

            double dist  = distance(nn_tri.nearest, point_set->points_[i]);
            double h_thr = correspondences_settings.w_distance_huber_value;
            if ( dist <= h_thr )
            {
                c.weight *= 1.0;
            }
            else
            {
                c.weight *= h_thr / dist;
            }
        }

        correspondences.push_back(c);
    }


    return true;
}


//-----------------------------------------------------------------------------


static bool
compute_correspondences_mesh2ps_GPU(Surface_mesh *template_mesh, geometry::Point_set *point_set, const Correspondences_settings &correspondences_settings, std::vector<Correspondence> &correspondences)
{
#ifdef HAVE_OCL
    if (template_mesh == NULL || point_set == NULL || correspondences_settings.constr_dir != surface_mesh::corresp_dir_mesh2ps)
    {
        return false;
    }

    // do not build Ematch correspondences for specified vertices or triangles with specified vertices
    Surface_mesh::Vertex_property<bool> vertex_omit = template_mesh->get_vertex_property<bool>("v:omit");
    Surface_mesh::Vertex_property<double> templ_local_weight = template_mesh->get_vertex_property<double>("v:fitting_weight");


    cl::CL_state* cl = cl::CL_state::instance();

    if (cl == NULL)
        return false;

    Surface_mesh::Vertex_property<Point> vpoint = template_mesh->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point> vnormal = template_mesh->get_vertex_property<Point>("v:normal");

    std::vector<Point>  &ps_points  = point_set->points_;
    std::vector<Normal> &ps_normals = point_set->normals_;

    if ((! vnormal || ps_normals.empty()) && correspondences_settings.filter_normals)
    {
        std::cout << "compute_correspondences_GPU: [ERROR] Filter normals is true and either mesh or point set have no vertex normals!"
                  << "mesh normals: " << (bool)vnormal << "   pointset normals: " << ps_normals.empty() << std::endl;
        return false;
    }


    const cl_context& ocl_context = cl->ocl_context().context();
    const cl_command_queue& cmdq  = cl->ocl_context().cmd_queue();



    cl_int err;
    size_t size;

    //START: create source buffers and upload data
    cl_mem clbuf_mesh_vertices, clbuf_ps_points;

    size = vpoint.vector().size() * sizeof(Point);
    clbuf_mesh_vertices = clCreateBuffer(ocl_context, CL_MEM_READ_ONLY, size, NULL, &err);
    err = clEnqueueWriteBuffer(cmdq, clbuf_mesh_vertices, CL_TRUE, 0, size, vpoint.data(), 0, NULL, NULL);

    size = ps_points.size() * sizeof(Point);
    clbuf_ps_points = clCreateBuffer(ocl_context, CL_MEM_READ_ONLY, size, NULL, &err);
    err = clEnqueueWriteBuffer(cmdq, clbuf_ps_points, CL_TRUE, 0, size, ps_points.data(), 0, NULL, NULL);
    //END: create source buffers and upload data


    //START: create result buffers
    cl_mem clbuf_result_closest_idx, clbuf_result_closest_dist;

    std::vector<unsigned int> result_closest_idx(vpoint.vector().size());
    std::vector<float>        result_closest_dist(vpoint.vector().size());

    const size_t result_closest_idx_size = result_closest_idx.size() * sizeof(unsigned int);
    clbuf_result_closest_idx = clCreateBuffer(ocl_context, CL_MEM_WRITE_ONLY, result_closest_idx_size, NULL, &err);

    const size_t result_closest_dist_size = result_closest_dist.size() * sizeof(float);
    clbuf_result_closest_dist = clCreateBuffer(ocl_context, CL_MEM_WRITE_ONLY, result_closest_dist_size, NULL, &err);
    //END: create results buffers

    const unsigned int num_vertices = (unsigned int) vpoint.vector().size();
    const unsigned int num_points   = (unsigned int) ps_points.size();

    cl_kernel mesh2ps = cl->kernel(cl::CORRESPONDENCES_MESH2PS_KERNEL);
    cl_uint idx = 0;

    err  = clSetKernelArg(mesh2ps, idx++, sizeof(cl_mem), &clbuf_mesh_vertices);
    err |= clSetKernelArg(mesh2ps, idx++, sizeof(cl_mem), &clbuf_ps_points);
    err |= clSetKernelArg(mesh2ps, idx++, sizeof(unsigned int), &num_vertices);
    err |= clSetKernelArg(mesh2ps, idx++, sizeof(unsigned int), &num_points);
    err |= clSetKernelArg(mesh2ps, idx++, sizeof(cl_mem), &clbuf_result_closest_idx);
    err |= clSetKernelArg(mesh2ps, idx++, sizeof(cl_mem), &clbuf_result_closest_dist);


    size_t local_work_size  = 32;
    size_t global_work_size = (vpoint.vector().size()/local_work_size + 1) * local_work_size;

    err = clEnqueueNDRangeKernel(cmdq, mesh2ps, 1, NULL, &global_work_size, &local_work_size, 0, NULL, NULL);

    clEnqueueReadBuffer(cmdq, clbuf_result_closest_idx,  CL_TRUE, 0, result_closest_idx_size , result_closest_idx.data(), 0, NULL, NULL);

    const float dist_until = (float)correspondences_settings.dist_until;
    const float normal_until = std::cos(correspondences_settings.normal_until * M_PI / 180.0);

    Correspondence c;
    c.constr_dir = corresp_dir_mesh2ps;
    c.on_template_bc = Vec3f(0.0f);

    for (size_t i=0; i < result_closest_idx.size(); ++i)
    {
        const Surface_mesh::Vertex v((int)i);

        c.on_template_vh       = v;
        c.distance_point2point = result_closest_dist[i];
        c.on_ps                = ps_points[ result_closest_idx[i] ];
        c.on_ps_n              = ps_normals[ result_closest_idx[i] ];
        c.on_template          = vpoint[v];
        c.on_template_n        = vnormal[v];

        // skip if nearest triangle has a specified vertex
        if (vertex_omit)
        {
            if (vertex_omit[v])
            {
                // std::cerr << "skip specified vertex !!!" << std::endl;
                continue;
            }
        }

        if (templ_local_weight)
        {
            if (templ_local_weight[v] < 1e-5)
            {
                // std::cerr << "skip since local weight is zero !!!" << std::endl;
                continue;
            }
        }


        if (correspondences_settings.filter_dist && c.distance_point2point > dist_until)
        {
            continue;
        }

        if (correspondences_settings.filter_normals && (dot(c.on_template_n, c.on_ps_n) < normal_until))
        {
            continue;
        }



        // weight
        c.weight        = 1.0;
        if (templ_local_weight)
        {
            c.weight *= templ_local_weight[v];
        }
        if ( correspondences_settings.w_distance_1r )
        {
            double dist = 100.0*std::numeric_limits<double>::epsilon() + c.distance_point2point;
            c.weight *= 1.0 / ( dist );
        }
        if ( correspondences_settings.w_distance_1ddMax )
        {
            std::cout << "w_distance_1ddMax not implemented" << std::endl;
            //c.weight *= ( 1.0 - ( c.distance_point2point / d_max_ps2mesh ) );
        }
        if ( correspondences_settings.w_dot_nn )
        {
            c.weight *= std::max( 0.0f, dot(c.on_template_n, c.on_ps_n) );
        }

        correspondences.push_back(c);
    }

    clReleaseMemObject(clbuf_mesh_vertices);
    clReleaseMemObject(clbuf_ps_points);
    clReleaseMemObject(clbuf_result_closest_idx);
    clReleaseMemObject(clbuf_result_closest_dist);

    return ! correspondences.empty();

#else
    std::cerr << "compute_correspondences_mesh2ps(): [ERROR] No OpenCL available!" << std::endl;
    return false;
#endif
}


//-----------------------------------------------------------------------------


static bool
compute_correspondences_ps2mesh_GPU(Surface_mesh* template_mesh, geometry::Point_set* point_set,  const Correspondences_settings& correspondences_settings, std::vector<Correspondence>& correspondences)
{
#ifdef HAVE_OCL
    if (template_mesh == NULL || point_set == NULL || correspondences_settings.constr_dir != surface_mesh::corresp_dir_ps2mesh)
    {
        return false;
    }

    // do not build Ematch correspondences for specified vertices or triangles with specified vertices
    Surface_mesh::Vertex_property<bool> vertex_omit = template_mesh->get_vertex_property<bool>("v:omit");
    Surface_mesh::Vertex_property<double> templ_local_weight = template_mesh->get_vertex_property<double>("v:fitting_weight");

    cl::CL_state* cl = cl::CL_state::instance();

    if (cl == NULL)
        return false;

    Surface_mesh::Vertex_property<Point> vpoint = template_mesh->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point> vnormal = template_mesh->get_vertex_property<Point>("v:normal");

    std::vector<Point>  &ps_points  = point_set->points_;
    std::vector<Normal> &ps_normals = point_set->normals_;

    if ((! vnormal || ps_normals.empty()) && correspondences_settings.filter_normals)
    {
        std::cout << "compute_correspondences_GPU: [ERROR] Filter normals is true and either mesh or point set have no vertex normals!"
                  << "mesh normals: " << (bool)vnormal << "   pointset normals: " << ps_normals.empty() << std::endl;
        return false;
    }


    const cl_context& ocl_context = cl->ocl_context().context();
    const cl_command_queue& cmdq  = cl->ocl_context().cmd_queue();



    cl_int err;
    size_t size;

    //START: create source buffers and upload data
    cl_mem clbuf_mesh_vertices, clbuf_ps_points;

    size = vpoint.vector().size() * sizeof(Point);
    clbuf_mesh_vertices = clCreateBuffer(ocl_context, CL_MEM_READ_ONLY, size, NULL, &err);
    err = clEnqueueWriteBuffer(cmdq, clbuf_mesh_vertices, CL_TRUE, 0, size, vpoint.data(), 0, NULL, NULL);

    size = ps_points.size() * sizeof(Point);
    clbuf_ps_points = clCreateBuffer(ocl_context, CL_MEM_READ_ONLY, size, NULL, &err);
    err = clEnqueueWriteBuffer(cmdq, clbuf_ps_points, CL_TRUE, 0, size, ps_points.data(), 0, NULL, NULL);
    //END: create source buffers and upload data


    //START: create result buffers
    cl_mem clbuf_result_closest_idx;

    std::vector<unsigned int> result_closest_idx(ps_points.size());

    const size_t result_closest_idx_size = result_closest_idx.size() * sizeof(unsigned int);
    clbuf_result_closest_idx = clCreateBuffer(ocl_context, CL_MEM_WRITE_ONLY, result_closest_idx_size, NULL, &err);
    //END: create results buffers

    const unsigned int num_vertices = (unsigned int) vpoint.vector().size();
    const unsigned int num_points   = (unsigned int) ps_points.size();

    cl_kernel ps2mesh = cl->kernel(cl::CORRESPONDENCES_PS2MESH_KERNEL);
    cl_uint idx = 0;

    err  = clSetKernelArg(ps2mesh, idx++, sizeof(cl_mem), &clbuf_mesh_vertices);
    err |= clSetKernelArg(ps2mesh, idx++, sizeof(cl_mem), &clbuf_ps_points);
    err |= clSetKernelArg(ps2mesh, idx++, sizeof(unsigned int), &num_vertices);
    err |= clSetKernelArg(ps2mesh, idx++, sizeof(unsigned int), &num_points);
    err |= clSetKernelArg(ps2mesh, idx++, sizeof(cl_mem), &clbuf_result_closest_idx);


    size_t local_work_size  = 32;
    size_t global_work_size = (ps_points.size()/local_work_size + 1) * local_work_size;

    err = clEnqueueNDRangeKernel(cmdq, ps2mesh, 1, NULL, &global_work_size, &local_work_size, 0, NULL, NULL);

    clEnqueueReadBuffer(cmdq, clbuf_result_closest_idx,  CL_TRUE, 0, result_closest_idx_size , result_closest_idx.data(), 0, NULL, NULL);

    Surface_mesh::Face_around_vertex_circulator fvc, fvc_end;
    Surface_mesh::Vertex_around_face_circulator vfc;

    Point np;
    Surface_mesh::Vertex v0,v1,v2;

    const float dist_until = (float)correspondences_settings.dist_until;
    const float normal_until = std::cos(correspondences_settings.normal_until * M_PI / 180.0);
    float nearest_dist, d;

    Correspondence c;
    c.constr_dir = corresp_dir_ps2mesh;

    for (size_t i=0; i < result_closest_idx.size(); ++i)
    {
        nearest_dist = FLT_MAX;

        fvc = fvc_end = template_mesh->faces(Surface_mesh::Vertex((int) result_closest_idx[i]));

        do
        {
            vfc = template_mesh->vertices(*fvc);
            const Point& p0 = vpoint[*(vfc)];++vfc;
            const Point& p1 = vpoint[*(vfc)];++vfc;
            const Point& p2 = vpoint[*(vfc)];

            d = geometry::dist_point_triangle(ps_points[i], p0, p1, p2, np);
            if (d < nearest_dist)
            {
                nearest_dist = d;
                c.face = *fvc;
                c.on_template = np;
            }
        }
        while(++fvc != fvc_end);


        vfc = template_mesh->vertices(c.face);
        v0 = *(vfc);
        v1 = *(++vfc);
        v2 = *(++vfc);


        // skip if nearest triangle has a specified vertex
        if (vertex_omit)
        {
            if (vertex_omit[v0] || vertex_omit[v1] || vertex_omit[v2])
            {
                // std::cerr << "skip specified vertex !!!" << std::endl;
                continue;
            }
        }

        if (templ_local_weight)
        {
            if (templ_local_weight[v0] < 1e-5 || templ_local_weight[v1] < 1e-5 || templ_local_weight[v2] < 1e-5)
            {
                // std::cerr << "skip since local weight is zero !!!" << std::endl;
                continue;
            }
        }


        const Point& p0 = vpoint[v0];
        const Point& p1 = vpoint[v1];
        const Point& p2 = vpoint[v2];

        const Normal& n0 = vnormal[v0];
        const Normal& n1 = vnormal[v1];
        const Normal& n2 = vnormal[v2];

        c.on_template_bc = geometry::barycentric_coordinates(c.on_template, p0, p1, p2);

        if (correspondences_settings.filter_dist && nearest_dist > dist_until)
        {
            continue;
        }

        c.on_template_n = c.on_template_bc[0]*n0 + c.on_template_bc[1]*n1 + c.on_template_bc[2]*n2;
        c.on_ps_n = ps_normals[i];

        if (correspondences_settings.filter_normals && (dot(c.on_template_n, c.on_ps_n) < normal_until))
        {
            continue;
        }

        // skip if target is on boundary
        if ( correspondences_settings.boundary_filter && is_point_boundary(*template_mesh, c.on_template, c.face))
        {
            continue;
        }

        c.distance_point2point = nearest_dist;
        c.on_ps = ps_points[i];

        // weight
        c.weight        = 1.0;
        if (templ_local_weight)
        {
            const double local_weight = c.on_template_bc[0]*templ_local_weight[v0] + c.on_template_bc[1]*templ_local_weight[v1] + c.on_template_bc[2]*templ_local_weight[v2];
            c.weight *= local_weight;
        }
        if ( correspondences_settings.w_distance_1r )
        {
            double dist = 100.0*std::numeric_limits<double>::epsilon() + c.distance_point2point;
            c.weight *= 1.0 / ( dist );
        }
        if ( correspondences_settings.w_distance_1ddMax )
        {
            std::cout << "w_distance_1ddMax not implemented" << std::endl;
            //c.weight *= ( 1.0 - ( c.distance_point2point / d_max_ps2mesh ) );
        }
        if ( correspondences_settings.w_dot_nn )
        {
            c.weight *= std::max( 0.0f, dot(c.on_template_n, c.on_ps_n) );
        }

        correspondences.push_back(c);
    }

    clReleaseMemObject(clbuf_mesh_vertices);
    clReleaseMemObject(clbuf_ps_points);
    clReleaseMemObject(clbuf_result_closest_idx);

    return ! correspondences.empty();

#else
    std::cerr << "compute_correspondences_ps2mesh(): [ERROR] No OpenCL available!" << std::endl;
    return false;
#endif
}


//-----------------------------------------------------------------------------


static bool
#ifdef HAVE_OCL
compute_correspondences(Surface_mesh *template_mesh, geometry::Point_set *point_set, const Correspondences_settings &correspondences_settings, std::vector<Correspondence> &correspondences, bool gpu = true)
#else
compute_correspondences(Surface_mesh *template_mesh, geometry::Point_set *point_set, const Correspondences_settings &correspondences_settings, std::vector<Correspondence> &correspondences, bool gpu = false)
#endif
{
    utility::Stop_watch watch;
    watch.start();
    correspondences.clear();

    bool success = false;

    switch(correspondences_settings.constr_dir)
    {
    case surface_mesh::corresp_dir_mesh2ps:
        if (gpu)
        {
            success = compute_correspondences_mesh2ps_GPU(template_mesh, point_set, correspondences_settings, correspondences);
        }
        else
        {
            success = compute_correspondences_mesh2ps_CPU(template_mesh, point_set, correspondences_settings, correspondences);
        }
        break;
    case surface_mesh::corresp_dir_ps2mesh:
        if (gpu)
        {
            success = compute_correspondences_ps2mesh_GPU(template_mesh, point_set, correspondences_settings, correspondences);
        }
        else
        {
            success = compute_correspondences_ps2mesh_CPU(template_mesh, point_set, correspondences_settings, correspondences);
        }
        break;
    case surface_mesh::corresp_dir_mesh2ps_and_ps2mesh:
        if (gpu)
        {
            success  = compute_correspondences_ps2mesh_GPU(template_mesh, point_set, correspondences_settings, correspondences);
            success &= compute_correspondences_mesh2ps_GPU(template_mesh, point_set, correspondences_settings, correspondences);
        }
        else
        {
            success  = compute_correspondences_ps2mesh_CPU(template_mesh, point_set, correspondences_settings, correspondences);
            success &= compute_correspondences_mesh2ps_CPU(template_mesh, point_set, correspondences_settings, correspondences);
        }
        break;
    }

    Surface_mesh::Vertex_property<Point> templ_points_after_comp_corr  = template_mesh->vertex_property<Point>("v:point_compcorr");
    Surface_mesh::Vertex_property<Point> vpoint = template_mesh->get_vertex_property<Point>("v:point");
    for (auto v : template_mesh->vertices())
    {
        templ_points_after_comp_corr[v] = vpoint[v];
    }



    // remove correspondences whose point-to-point distance is larger than some multiple of the standard deviation of distances
    if ( correspondences_settings.std_dev_filter )
    {
        // compute mean distance
        double mean_distance(0.0);
        for (unsigned int i = 0; i < correspondences.size(); ++i)
        {
            mean_distance += correspondences[i].distance_point2point;
        }
        mean_distance /= correspondences.size();
        // compute standard deviation
        double sigma(0.0);
        for (unsigned int i = 0; i < correspondences.size(); ++i)
        {
            sigma += pow( correspondences[i].distance_point2point - mean_distance, 2 );
        }
        sigma /= ( correspondences.size() - 1 );
        sigma  = sqrt(sigma);

        std::vector<Correspondence> filtered_correspondences;
        for (unsigned int i = 0; i < correspondences.size(); ++i)
        {
            if ( correspondences[i].distance_point2point < ( mean_distance + sigma * correspondences_settings.std_dev_factor ) )
            {
                filtered_correspondences.push_back( correspondences[i] );
            }
        }
        correspondences = filtered_correspondences;
    }

    // remove worst n% (w.r.t. point 2 point distance)
    if ( correspondences_settings.worst_nPerc_filter )
    {
        // sort correspondences w.r.t. point 2 point distance
        // smallest distance: front
        // largest distance: back
        std::sort( correspondences.begin() , correspondences.end() );

        std::vector<Correspondence> filtered_correspondences;
        for (unsigned int i = 0; i < (unsigned int) ( (1.0 - correspondences_settings.nPerc_worst / 100.0) * correspondences.size() ); ++i)
        {
            filtered_correspondences.push_back( correspondences[i] );
        }
        correspondences = filtered_correspondences;
    }

    std::cout << "Compute correspondences took " << watch.stop()/1000.0 << " s. Found " << correspondences.size() << " correspondences." << std::endl;

    if (correspondences.empty())
    {
        std::cerr << "Template_fit::compute_correspondences(): [ERROR] No correspondences found!" << std::endl;
        return false;
    }

    return success;
}


//-----------------------------------------------------------------------------


static
void
skin_correspondences(const Surface_mesh* mesh, const std::vector<Correspondence>& corr_in, std::vector<Correspondence>& corr_out,  bool inverse_transform = false, character::Skinning_mode skin_mode = character::LINEAR_BLENDING)
{
    Surface_mesh::Mesh_property< std::vector<Mat4f> > skinning_scanpose_to_tpose = mesh->get_mesh_property< std::vector<Mat4f> >("m:skinning_scanpose_to_tpose");
    Surface_mesh::Vertex_property<Vec4f> vdepends = mesh->get_vertex_property<Vec4f>("v:skin_depend");
    Surface_mesh::Vertex_property<Vec4f> vweights = mesh->get_vertex_property<Vec4f>("v:skin_weight");
    Surface_mesh::Vertex_property<Vec4f> vdepends2 = mesh->get_vertex_property<Vec4f>("v:skin_depend2");
    Surface_mesh::Vertex_property<Vec4f> vweights2 = mesh->get_vertex_property<Vec4f>("v:skin_weight2");

    corr_out.clear();

    if (!(vdepends && vweights && vdepends2 && vweights2 && skinning_scanpose_to_tpose))
    {
        //std::cout << "my_helper.h - skin_correspondences: [ERROR] Either no depends/weights or skinning matrices present in mesh." << std::endl;
        return;
    }

    std::vector<Mat4f> &skin_mats = skinning_scanpose_to_tpose[0];

    Surface_mesh::Vertex_property<Point> vpoint = mesh->get_vertex_property<Point>("v:point");

    int k,depend,idx;
    size_t i;
    Mat4f blend_mat;
    Vec3f p_mesh, p_ps;
    Surface_mesh::Vertex v;
    Surface_mesh::Vertex_around_face_circulator vfc,vfc_end;

    corr_out = corr_in;


    if (skin_mode == character::DUALQUAT_BLENDING)
    {
        DQuatf blend_dq;
        std::vector<DQuatf> dqs(skin_mats.size());
        //prepare dualquat skinning
        if (inverse_transform)
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                mat4_to_dualquat(inverse(skin_mats[i]), dqs[i]);
            }
        }
        else
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                mat4_to_dualquat(skin_mats[i], dqs[i]);
            }
        }


        for (i=0; i < corr_in.size(); ++i)
        {
            Correspondence&       c_out = corr_out[i];

            if (c_out.constr_dir == corresp_dir_ps2mesh)
            {
                vfc = vfc_end = mesh->vertices(c_out.face);
                idx = 0;

                do
                {
                    v = *vfc;



                    for (k=0; k < 4; ++k)
                    {
                        depend = (int)vdepends[v][k];
                        DQuatf& tmp_dq = dqs[depend];
                        if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                        {
                            blend_dq += tmp_dq * vweights[v][k];
                        } else
                        {
                            blend_dq -= tmp_dq * vweights[v][k];
                        }
                    }

                    for (k=0; k < 4; ++k)
                    {
                        depend = (int)vdepends2[v][k];
                        DQuatf& tmp_dq = dqs[depend];
                        if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                        {
                            blend_dq += tmp_dq * vweights2[v][k];
                        } else
                        {
                            blend_dq -= tmp_dq * vweights2[v][k];
                        }
                    }

                    blend_dq = normalize(blend_dq);

                    dq_to_mat4(blend_dq, blend_mat);

                    if (idx == 0)
                    {
                        p_mesh  = affine_transform(blend_mat, vpoint[v])   * c_out.on_template_bc[idx];
                        p_ps    = affine_transform(blend_mat, c_out.on_ps) * c_out.on_template_bc[idx];
                    }
                    else
                    {
                        p_mesh += affine_transform(blend_mat, vpoint[v])   * c_out.on_template_bc[idx];
                        p_ps   += affine_transform(blend_mat, c_out.on_ps) * c_out.on_template_bc[idx];
                    }

                    ++idx;
                }
                while(++vfc != vfc_end);


                //transform vertex and normal
                c_out.on_ps = p_ps;
                const Mat3f normal_mat = Mat3f(blend_mat);
                c_out.on_ps_n = normalize(normal_mat * c_out.on_ps_n);
                //c_out.on_template = p_mesh;
            }
            else if (c_out.constr_dir == corresp_dir_mesh2ps)
            {
                v = c_out.on_template_vh;

                blend_dq = DQuatf(0,0,0,0,0,0,0,0);
                for (k=0; k < 4; ++k)
                {
                    depend = (int)vdepends[v][k];
                    DQuatf& tmp_dq = dqs[depend];
                    if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                    {
                        blend_dq += tmp_dq * vweights[v][k];
                    } else
                    {
                        blend_dq -= tmp_dq * vweights[v][k];
                    }
                }

                blend_dq = normalize(blend_dq);

                dq_to_mat4(blend_dq, blend_mat);

                //transform vertex and normal
                c_out.on_ps = affine_transform(blend_mat, c_out.on_ps);
                const Mat3f normal_mat = Mat3f(blend_mat);
                c_out.on_ps_n = normalize(normal_mat * c_out.on_ps_n);
                //c_out.on_template = affine_transform(blend_mat, c_out.on_template);
            }
            else
            {
                std::cerr << "my_helper.h - skin_correspondences: [WARNING] Skinning of correspondences in both directions not implemented." << std::endl;
            }
        }
    }
    else if (skin_mode == character::LINEAR_BLENDING)
    {
        for (i=0; i < corr_in.size(); ++i)
        {
            Correspondence& c_out = corr_out[i];
            if (c_out.constr_dir == corresp_dir_ps2mesh)
            {
                vfc = vfc_end = mesh->vertices(c_out.face);
                idx = 0;

                do
                {
                    v = *vfc;

                    blend_mat  = vweights[v][0] * skin_mats[(int)vdepends[v][0]];
                    blend_mat += vweights[v][1] * skin_mats[(int)vdepends[v][1]];
                    blend_mat += vweights[v][2] * skin_mats[(int)vdepends[v][2]];
                    blend_mat += vweights[v][3] * skin_mats[(int)vdepends[v][3]];

                    blend_mat += vweights2[v][0] * skin_mats[(int)vdepends2[v][0]];
                    blend_mat += vweights2[v][1] * skin_mats[(int)vdepends2[v][1]];
                    blend_mat += vweights2[v][2] * skin_mats[(int)vdepends2[v][2]];
                    blend_mat += vweights2[v][3] * skin_mats[(int)vdepends2[v][3]];

                    if (idx == 0)
                    {
                        p_mesh  = affine_transform(blend_mat, vpoint[v])   * c_out.on_template_bc[idx];
                        p_ps    = affine_transform(blend_mat, c_out.on_ps) * c_out.on_template_bc[idx];
                    }
                    else
                    {
                        p_mesh += affine_transform(blend_mat, vpoint[v])   * c_out.on_template_bc[idx];
                        p_ps   += affine_transform(blend_mat, c_out.on_ps) * c_out.on_template_bc[idx];
                    }

                    ++idx;
                }
                while(++vfc != vfc_end);


                //transform vertex and normal
                c_out.on_ps = p_ps;
                const Mat3f normal_mat = Mat3f(blend_mat);
                c_out.on_ps_n = normalize(normal_mat * c_out.on_ps_n);
                //c_out.on_template = p_mesh;
            }
            else if (c_out.constr_dir == corresp_dir_mesh2ps)
            {
                v = c_out.on_template_vh;

                blend_mat  = vweights[v][0] * skin_mats[(int)vdepends[v][0]];
                blend_mat += vweights[v][1] * skin_mats[(int)vdepends[v][1]];
                blend_mat += vweights[v][2] * skin_mats[(int)vdepends[v][2]];
                blend_mat += vweights[v][3] * skin_mats[(int)vdepends[v][3]];

                blend_mat += vweights2[v][0] * skin_mats[(int)vdepends2[v][0]];
                blend_mat += vweights2[v][1] * skin_mats[(int)vdepends2[v][1]];
                blend_mat += vweights2[v][2] * skin_mats[(int)vdepends2[v][2]];
                blend_mat += vweights2[v][3] * skin_mats[(int)vdepends2[v][3]];

                //transform vertex and normal
                c_out.on_ps = affine_transform(blend_mat, c_out.on_ps);
                const Mat3f normal_mat = Mat3f(blend_mat);
                c_out.on_ps_n = normalize(normal_mat * c_out.on_ps_n);
                //c_out.on_template = affine_transform(blend_mat, c_out.on_template);
            }
            else
            {
                std::cerr << "my_helper.h - skin_correspondences: [WARNING] Skinning of correspondences in both directions not implemented." << std::endl;
            }
        }
    }
}


//-----------------------------------------------------------------------------


static
void
skin_pointset_landmarks(const Surface_mesh* mesh, const geometry::Point_set* pointset, const Landmarks_manager& lm_manager, std::vector<Point>& landmarks_out, bool inverse_transform = false, character::Skinning_mode skin_mode = character::LINEAR_BLENDING)
{


    Surface_mesh::Mesh_property< std::vector<Mat4f> > skinning_scanpose_to_tpose = mesh->get_mesh_property< std::vector<Mat4f> >("m:skinning_scanpose_to_tpose");
    Surface_mesh::Vertex_property<Vec4f> vdepends = mesh->get_vertex_property<Vec4f>("v:skin_depend");
    Surface_mesh::Vertex_property<Vec4f> vweights = mesh->get_vertex_property<Vec4f>("v:skin_weight");
    Surface_mesh::Vertex_property<Vec4f> vdepends2 = mesh->get_vertex_property<Vec4f>("v:skin_depend2");
    Surface_mesh::Vertex_property<Vec4f> vweights2 = mesh->get_vertex_property<Vec4f>("v:skin_weight2");

    landmarks_out.clear();

    if (!(vdepends && vweights && vdepends2 && vweights2 && skinning_scanpose_to_tpose))
    {
        return;
    }

    const std::vector<Mat4f> &skin_mats = skinning_scanpose_to_tpose[0];

    size_t i;
    int k,depend;
    Mat4f blend_mat;
    Surface_mesh::Vertex v;
    std::vector<Surface_mesh::Vertex> landmarks_idcs_template;

    //get current non-skinned landmarks
    lm_manager.get_landmarks_point_set_stdvec(pointset, landmarks_out);
    //get indices of landmarks from mesh
    lm_manager.get_indices_template(mesh, landmarks_idcs_template);

    if (skin_mode == character::DUALQUAT_BLENDING)
    {
        DQuatf blend_dq;
        std::vector<DQuatf> dqs(skin_mats.size());
        //prepare dualquat skinning
        if (inverse_transform)
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                mat4_to_dualquat(inverse(skin_mats[i]), dqs[i]);
            }
        }
        else
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                mat4_to_dualquat(skin_mats[i], dqs[i]);
            }
        }


        for (i=0; i < landmarks_idcs_template.size(); ++i)
        {
            v = landmarks_idcs_template[i];


            blend_dq = DQuatf(0,0,0,0,0,0,0,0);
            for (k=0; k < 4; ++k)
            {
                depend = (int)vdepends[v][k];
                DQuatf& tmp_dq = dqs[depend];
                if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                {
                    blend_dq += tmp_dq * vweights[v][k];
                } else
                {
                    blend_dq -= tmp_dq * vweights[v][k];
                }
            }
            for (k=0; k < 4; ++k)
            {
                depend = (int)vdepends2[v][k];
                DQuatf& tmp_dq = dqs[depend];
                if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                {
                    blend_dq += tmp_dq * vweights2[v][k];
                } else
                {
                    blend_dq -= tmp_dq * vweights2[v][k];
                }
            }

            blend_dq = normalize(blend_dq);

            dq_to_mat4(blend_dq, blend_mat);

            landmarks_out[i] = affine_transform(blend_mat, landmarks_out[i]);
        }
    }
    else if (skin_mode == character::LINEAR_BLENDING)
    {

        for (i=0; i < landmarks_idcs_template.size(); ++i)
        {
            v = landmarks_idcs_template[i];

            blend_mat  = vweights[v][0] * skin_mats[(int)vdepends[v][0]];
            blend_mat += vweights[v][1] * skin_mats[(int)vdepends[v][1]];
            blend_mat += vweights[v][2] * skin_mats[(int)vdepends[v][2]];
            blend_mat += vweights[v][3] * skin_mats[(int)vdepends[v][3]];

            blend_mat += vweights2[v][0] * skin_mats[(int)vdepends2[v][0]];
            blend_mat += vweights2[v][1] * skin_mats[(int)vdepends2[v][1]];
            blend_mat += vweights2[v][2] * skin_mats[(int)vdepends2[v][2]];
            blend_mat += vweights2[v][3] * skin_mats[(int)vdepends2[v][3]];

            landmarks_out[i] = affine_transform(blend_mat, landmarks_out[i]);
        }
    }


}

static
void skin_mesh(const std::string& prop_name, Surface_mesh* mesh, bool inverse_transform = false, character::Skinning_mode skin_mode = character::LINEAR_BLENDING)
{

    Surface_mesh::Mesh_property< std::vector<Mat4f> > skinning_mats = mesh->get_mesh_property< std::vector<Mat4f> >(prop_name);

    Surface_mesh::Vertex_property<Point> vpoints  = mesh->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Vec4f> vdepends = mesh->get_vertex_property<Vec4f>("v:skin_depend");
    Surface_mesh::Vertex_property<Vec4f> vweights = mesh->get_vertex_property<Vec4f>("v:skin_weight");
    Surface_mesh::Vertex_property<Vec4f> vdepends2 = mesh->get_vertex_property<Vec4f>("v:skin_depend2");
    Surface_mesh::Vertex_property<Vec4f> vweights2 = mesh->get_vertex_property<Vec4f>("v:skin_weight2");

    if (!(vdepends && vweights && vdepends2 && vweights2 && vpoints && skinning_mats))
    {
        //std::cerr << "my_helper.h - skin_mesh: [ERROR] No depends/weights/points in mesh or no skinning matrices with property name \"" << prop_name << "\"." << std::endl;
        return;
    }

    std::vector<Mat4f> skin_mats = skinning_mats[0];

    Surface_mesh::Vertex_property<Normal> vnormals  = mesh->get_vertex_property<Normal>("v:normal");

    if (skin_mode == character::DUALQUAT_BLENDING)
    {
        int k,depend;
        size_t i;
        DQuatf blend_dq;
        Mat4f blend_mat;
        std::vector<DQuatf> dqs(skin_mats.size());

        //prepare dualquat skinning
        if (inverse_transform)
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                mat4_to_dualquat(inverse(skin_mats[i]), dqs[i]);
            }
        }
        else
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                mat4_to_dualquat(skin_mats[i], dqs[i]);
            }
        }


        const std::vector<Vec4f>& w = vweights.vector();
        const std::vector<Vec4f>& d = vdepends.vector();
        const std::vector<Vec4f>& w2 = vweights2.vector();
        const std::vector<Vec4f>& d2 = vdepends2.vector();

        for (i=0; i < vpoints.vector().size(); ++i)
        {

            blend_dq = DQuatf(0,0,0,0,0,0,0,0);
            for (k=0; k < 4; ++k)
            {
                depend = (int)d[i][k];
                DQuatf& tmp_dq = dqs[depend];
                if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                {
                    blend_dq += tmp_dq * w[i][k];
                } else
                {
                    blend_dq -= tmp_dq * w[i][k];
                }
            }
            for (k=0; k < 4; ++k)
            {
                depend = (int)d2[i][k];
                DQuatf& tmp_dq = dqs[depend];
                if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                {
                    blend_dq += tmp_dq * w2[i][k];
                } else
                {
                    blend_dq -= tmp_dq * w2[i][k];
                }
            }

            blend_dq = normalize(blend_dq);

            dq_to_mat4(blend_dq, blend_mat);

            //transform vertex and normal
            vpoints.vector()[i] = affine_transform(blend_mat, vpoints.vector()[i]);
            if (vnormals)
            {
                vnormals.vector()[i] = normalize(Mat3f(blend_mat) * vnormals.vector()[i]);
            }
        }
    }
    else if (skin_mode == character::LINEAR_BLENDING)
    {
        size_t i;
        Mat4f blendmat;

        if (inverse_transform)
        {
            for (i=0; i < skin_mats.size(); ++i)
            {
                skin_mats[i] = inverse(skin_mats[i]);
            }
        }

        const std::vector<Vec4f>& w = vweights.vector();
        const std::vector<Vec4f>& d = vdepends.vector();
        const std::vector<Vec4f>& w2 = vweights2.vector();
        const std::vector<Vec4f>& d2 = vdepends2.vector();

        for (i=0; i < vpoints.vector().size(); ++i)
        {
            blendmat  = w[i][0] * skin_mats[(int)d[i][0]];
            blendmat += w[i][1] * skin_mats[(int)d[i][1]];
            blendmat += w[i][2] * skin_mats[(int)d[i][2]];
            blendmat += w[i][3] * skin_mats[(int)d[i][3]];

            blendmat += w2[i][0] * skin_mats[(int)d2[i][0]];
            blendmat += w2[i][1] * skin_mats[(int)d2[i][1]];
            blendmat += w2[i][2] * skin_mats[(int)d2[i][2]];
            blendmat += w2[i][3] * skin_mats[(int)d2[i][3]];

            vpoints.vector()[i] = affine_transform(blendmat, vpoints.vector()[i]);
            if (vnormals)
            {
                vnormals.vector()[i] = normalize(Mat3f(blendmat) * vnormals.vector()[i]);
            }
        }

    }


}


static
void get_pose_vector(const character::Skeleton& s, std::vector<Mat4f>& pose_vec)
{
    character::Joint* j;
    pose_vec.resize(s.joints_.size());

    Mat4f m1,m2;

    for (size_t i=0; i < s.joints_.size(); ++i)
    {
        j = s.joints_[i];

        m1 = j->local_;
        m1[12] = m1[13] = m1[14] = 0.0f;
        m2 = j->bind_pose_local_;
        m2[12] = m2[13] = m2[14] = 0.0f;

        pose_vec[i] = transpose(m2) * m1;
    }
}

static
void set_pose_vector(const std::vector<Mat4f>& pose_vec, character::Skeleton& s)
{
    s.reset_to_bindpose();

    character::Joint* j;

    for (size_t i=0; i < s.joints_.size(); ++i)
    {
        j = s.joints_[i];

        j->local_ = j->local_ * pose_vec[i];
    }
    s.update();
}

static
void compute_local_vertex_rotation(Surface_mesh* mesh, std::vector<Mat3f>& local_vrot)
{
    Surface_mesh::Vertex_property<Point> points = mesh->get_vertex_property<Point>("v:point");

    if (!points)
        return;


    Surface_mesh::Halfedge_around_vertex_circulator hvc;
    Point p1,p2,p3;
    Vec3f v1,v2;
    Vec3f x,y,z;

    Surface_mesh::Vertex_iterator vit;
    for (vit = mesh->vertices_begin(); vit != mesh->vertices_end(); ++vit)
    {
        Mat3f& m = local_vrot[(*vit).idx()];

        hvc = mesh->halfedges(*vit);

        p1 = points[*vit];
        p2 = points[mesh->to_vertex(*hvc)];
        ++hvc;
        p3 = points[mesh->to_vertex(*hvc)];


        v1 = normalize(p1 - p2);
        v2 = normalize(p1 - p3);

        x = v1;
        y = normalize(cross(x,v2));
        z = normalize(cross(x,y));

        m[0] = x[0]; m[1] = y[0]; m[2] = z[0];
        m[3] = x[1]; m[4] = y[1]; m[5] = z[1];
        m[6] = x[2]; m[7] = y[2]; m[8] = z[2];
    }

}

} //namespace surface_mesh
} //namespace graphene
//=============================================================================
#endif // GRAPHENE_MYHELPER_H
//=============================================================================
