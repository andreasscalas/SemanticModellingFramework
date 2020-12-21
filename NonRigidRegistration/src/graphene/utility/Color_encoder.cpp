//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


#include <graphene/types.h>
#include <graphene/utility/Color_encoder.h>


//=============================================================================


namespace graphene {
namespace utility {


//=============================================================================


void
Color_encoder::
set_range(float _min, float _max, bool _signed)
{
    if (_min == _max)
    {
        val0_ = val1_ = val2_ = val3_ = val4_ = _min;
    }
    else
    {
        if (_min > _max) std::swap(_min, _max);
        val0_ = _min + 0.0/4.0 * (_max - _min);
        val1_ = _min + 1.0/4.0 * (_max - _min);
        val2_ = _min + 2.0/4.0 * (_max - _min);
        val3_ = _min + 3.0/4.0 * (_max - _min);
        val4_ = _min + 4.0/4.0 * (_max - _min);
    }
    signed_mode_ = _signed;
}


//-----------------------------------------------------------------------------


Color
Color_encoder::
color_unsigned(float _v) const
{
    if (val4_ <= val0_) return  Color(0, 0, 255);

    unsigned char u;

    if (_v < val0_) return  Color(0, 0, 255);
    if (_v > val4_) return  Color(255, 0, 0);


    if (_v <= val2_)
    {
        // [v0, v1]
        if (_v <= val1_)
        {
            u = (unsigned char) (255.0 * (_v - val0_) / (val1_ - val0_));
            return  Color(0, u, 255);
        }
        // ]v1, v2]
        else
        {
            u = (unsigned char) (255.0 * (_v - val1_) / (val2_ - val1_));
            return  Color(0, 255, 255-u);
        }
    }
    else
    {
        // ]v2, v3]
        if (_v <= val3_)
        {
            u = (unsigned char) (255.0 * (_v - val2_) / (val3_ - val2_));
            return  Color(u, 255, 0);
        }
        // ]v3, v4]
        else
        {
            u = (unsigned char) (255.0 * (_v - val3_) / (val4_ - val3_));
            return  Color(255, 255-u, 0);
        }
    }
}


//-----------------------------------------------------------------------------


Color
Color_encoder::
color_signed(float _v) const
{
    if (val4_ <= val0_) return  Color(0,255,0);

    unsigned char r,g,b;

    if      (_v < val0_) _v = val0_;
    else if (_v > val4_) _v = val4_;

    if (_v < 0.0)
    {
        r = val0_ ? (unsigned char)(255.0 * _v / val0_) : 0;
        b = 0;
    }
    else
    {
        r = 0;
        b = val4_ ? (unsigned char)(255.0 * _v / val4_) : 0;
    }
    g = 255 - r - b;

    return  Color(r, g, b);
}


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
