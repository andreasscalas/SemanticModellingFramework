//=============================================================================

#ifndef PLANE_H
#define PLANE_H


//== INCLUDES =================================================================

#include <graphene/geometry/Vector.h>


//== NAMESPACES ===============================================================


namespace graphene {
namespace geometry {


//== CLASS DEFINITION =========================================================


/**
 This class stores a plane in normal form and provides useful
 algorithms for left/right test, itersection, and so on.
**/

class Plane
{
public:

  /// typedefs
  typedef Vector<Scalar, 3>  Vec3;
  typedef Vector<Scalar, 4>  Vec4;


  /// constructor: coefficients
  Plane( Scalar _a=0, Scalar _b=0, Scalar _c=0, Scalar _d=0 )
  : coeffs_(_a, _b, _c, _d)
  { HNF(); }


  /// constructor: origin and normal
  Plane( const Vec3& _o, const Vec3& _n )
      : coeffs_(_n[0], _n[1], _n[2], -dot(_n,_o))
  { HNF(); }


  /// constructor: 3 points
  Plane( const Vec3& _v0, const Vec3& _v1, const Vec3& _v2 )
  {
    Vec3 n = cross((_v1-_v0),(_v2-_v0));
    coeffs_ = Vec4(n[0], n[1], n[2], -dot(n,_v0));
    HNF();
  }


  /// normal vector
  Vec3 normal() const { return Vec3(coeffs_[0], coeffs_[1], coeffs_[2]); }


  /// coeffitients
  const Vec4& coeffs() const { return coeffs_; }


  /// signed distance point-plane
  Scalar distance( const Vec3& _v ) const
  {
    return (_v[0]*coeffs_[0] +
            _v[1]*coeffs_[1] +
            _v[2]*coeffs_[2] +
                  coeffs_[3] );
  }


  /// predicate: above plane
  bool operator() ( const Vec3& _v ) const { return distance(_v) > 0; }



private:

  void HNF()
  {
      Scalar n = norm(normal());
      if (n != 0.0) coeffs_ /= n;
  }


private:

  Vec4 coeffs_;
};


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

