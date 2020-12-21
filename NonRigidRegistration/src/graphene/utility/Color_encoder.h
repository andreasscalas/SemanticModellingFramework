//=============================================================================


#ifndef GRAPHENE_COLOR_ENCODER_H
#define GRAPHENE_COLOR_ENCODER_H


//=============================================================================


#include <graphene/geometry/Vector.h>
#include <graphene/types.h>


//=============================================================================


namespace graphene {
namespace utility {


//=============================================================================

/// \addtogroup utility
/// The utility module provides commonly used utility functions.


/// \addtogroup utility
/// @{


//=============================================================================

/// A simple color encoder
class Color_encoder
{

public:

    Color_encoder(float _min=0.0, float _max=1.0, bool _signed=false)
    {
        set_range(_min, _max, _signed);
    };

    void set_range(float _min, float _max, bool _signed);

    Vec3f color(float _v) const
    {
        return signed_mode_ ? color_signed(_v) : color_unsigned(_v);
    };

    float min() const { return val0_; };
    float max() const { return val4_; };

private:

    Vec3f color_unsigned(float _v) const;

    Vec3f color_signed(float _v) const;

    float  val0_, val1_, val2_, val3_, val4_;
    bool   signed_mode_;
};

//=============================================================================
/// @}
//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_COLOR_ENCODER_H
//=============================================================================
