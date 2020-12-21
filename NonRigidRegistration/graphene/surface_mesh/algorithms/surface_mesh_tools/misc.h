//=============================================================================
// Copyright (C) 2014 Graphics & Geometry Group, Bielefeld University
//=============================================================================


#ifndef GRAPHENE_SURFACE_MESH_TOOLS_MISC_H
#define GRAPHENE_SURFACE_MESH_TOOLS_MISC_H


//=============================================================================


#include <graphene/types.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>

#include <Eigen/Dense>


//=============================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================


/// \addtogroup surface_mesh
/// @{


//=============================================================================


/// compute mean edge length of a surface mesh
Scalar mean_edge_length(const Surface_mesh& mesh);


/// compute centroid of a face
Point centroid(Surface_mesh& mesh, const Surface_mesh::Face& f);


bool load_selection_from_file(std::vector<int>& output, const std::string& filename);

bool load_selection_from_file(std::vector<unsigned int>& output, const std::string& filename);

bool load_weights_from_file(std::vector<double>& output, const std::string& filename);

bool load_selection_from_file(std::vector<size_t>& output, const std::string& filename);

bool load_selection_from_file(std::vector<Surface_mesh::Vertex>& output, const std::string& filename);

bool load_selection_from_file_with_weights(std::vector<unsigned int>& indices,
                                           std::vector<double>&       local_weights,
                                           const std::string&         filename);

bool intersect_ray_triangle( const Point& tri_p0, const Point& tri_p1, const Point& tri_p2, // triangle points
                             const Point& ray_center, const Vec3d& ray_direction, // ray
                             const bool allow_negative_t, // allow negative t, i.e., an intersection opposite to the ray
                             Point&  p_intersect,    // out: intersection point
                             double& t_intersect ); // out: intersection parameter t

bool intersect_ray_triangle( const Point& tri_p0, const Point& tri_p1, const Point& tri_p2, // triangle points
                             const Normal& tri_n,
                             const Point& ray_center, const Vec3d& ray_direction, // ray
                             const bool allow_negative_t, // allow negative t, i.e., an intersection opposite to the ray
                             Point&  p_intersect,    // out: intersection point
                             Point&  n_intersect,    // out: intersection normal
                             double& t_intersect ); // out: intersection parameter t

bool intersect_ray_plane( const Point& plane_p0, // one (arbitrary) point on the plane
                          const Normal& plane_n0, // normal vector of the plane
                          const Point& ray_center, const Vec3d& ray_direction, // ray
                          Point& p_intersect,    // out: intersection point
                          double& t_intersect ); // out: intersection parameter t

bool write_eigen_matrix_as_bin(const Eigen::MatrixXd& eigen_matrix,
                               const std::string& filename,
                               const unsigned int rows,
                               const unsigned int cols);

bool write_eigen_vector_as_bin(const Eigen::VectorXd& eigen_vector,
                               const std::string& filename,
                               const unsigned int dim);

bool read_eigen_matrix_from_bin(Eigen::MatrixXd& eigen_matrix,
                                const std::string& filename);

bool read_eigen_vector_from_bin(Eigen::VectorXd& eigen_vector,
                                const std::string& filename);


//=============================================================================
/// @}
//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SURFACE_MESH_TOOLS_MISC_H
//=============================================================================
