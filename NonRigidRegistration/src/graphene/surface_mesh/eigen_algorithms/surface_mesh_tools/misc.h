//=============================================================================
// Copyright (C) 2014 Graphics & Geometry Group, Bielefeld University
//=============================================================================


#ifndef GRAPHENE_SURFACE_MESH_TOOLS_MISC_H
#define GRAPHENE_SURFACE_MESH_TOOLS_MISC_H


//=============================================================================


#include <graphene/types.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>


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


//=============================================================================
/// @}
//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SURFACE_MESH_TOOLS_MISC_H
//=============================================================================
