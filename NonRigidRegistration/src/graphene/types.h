//=============================================================================

#ifndef GRAPHENE_TYPES_H
#define GRAPHENE_TYPES_H


//== INCLUDES =================================================================

#include <graphene/macros.h>
#include <graphene/geometry/Vector.h>


//=============================================================================

namespace graphene {


//=============================================================================


/// Scalar type
typedef float Scalar;

/// Point type
typedef Vector<Scalar,3> Point;

/// 3D vector type
typedef Vector<Scalar,3> Vec3;

/// Normal type
typedef Vector<Scalar,3> Normal;

/// Color type
typedef Vector<Scalar,3> Color;

/// Pixel coordinate type
typedef Vector<Scalar,2> Pixel_coordinate;

/// Texture coordinate type
typedef Vector<Scalar,3> Texture_coordinate;

/// Common shorthand for unsigned int
// typedef unsigned int uint;   // Don't use it!! - Qt's definition of uint will collide with that on Windows.

/// Markers to specify fixed, deformable and handle regions for deformation
/// modules, as well as vertices to be kept planar when using constraints.
enum Region_type { fixed, deformable, handle, planar, circular, feature, no_region };


//=============================================================================
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_TYPES_H
//============================================================================
