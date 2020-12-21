#ifndef GRAPHENE_SQRT3_SUBDIVISION_H
#define GRAPHENE_SQRT3_SUBDIVISION_H

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>

namespace graphene {
namespace surface_mesh {

/// \addtogroup subdivision subdivision
/// @{

/// Perform one step of sqrt3 subdivision on Surface_mesh \a mesh.
void sqrt3_subdivision(Surface_mesh& mesh);

/// @}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_SQRT3_SUBDIVISION_H
//=============================================================================
