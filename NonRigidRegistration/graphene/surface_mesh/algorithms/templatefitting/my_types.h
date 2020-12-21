//=============================================================================
#ifndef GRAPHENE_MY_TYPES_H
#define GRAPHENE_MY_TYPES_H
//=============================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <graphene/surface_mesh/algorithms/templatefitting/types/Correspondence.h>
#include <graphene/surface_mesh/algorithms/templatefitting/types/Correspondences_settings.h>
#include <graphene/surface_mesh/algorithms/templatefitting/types/Logging_settings.h>

//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================


typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double>      Tripl;


enum Camera_type { camera_type_photoscan, camera_type_ourphotoscanner };




//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_MY_TYPES_H
//=============================================================================
