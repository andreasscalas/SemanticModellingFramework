//=============================================================================

#ifndef CATMULL_CLARK_SUBDIVISION_H
#define CATMULL_CLARK_SUBDIVISION_H


//== INCLUDES =================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>


//== NAMESPACE ================================================================

namespace graphene {
namespace surface_mesh {


//=============================================================================


/// \addtogroup subdivision subdivision
/// @{

/// Perform one step of Loop subdivision on Surface_mesh \a mesh.
void catmull_clark_subdivision(Surface_mesh& mesh);

/// @}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
