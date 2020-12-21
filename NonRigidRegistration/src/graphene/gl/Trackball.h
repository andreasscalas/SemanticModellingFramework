/*!
 * \file Trackball.h
 * \author Thomas Waltemate
 */

#ifndef ICSPACE_TRACKBALL_H
#define ICSPACE_TRACKBALL_H

#ifndef PI_float
#define PI_float 3.14159265358979323846f
#endif

#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Matrix4x4.h>

namespace graphene
{
namespace gl
{


/*!
  \brief Provides methods for intuitive transformation of the scene.
*/

struct Trackball
{
private:
    ///window width and height
    int width_, height_;

    /// last x coordinate
    int last_x_;
    /// last y coordinate
    int last_y_;
    /// last z coordinate
    int last_z_;
    /// start point in 3D
    Vec3f start_point_3D_;
    /// radius of the trackball
    float radius_;
    /// start point for translation
    Vec3f start_point_translation_;


    /// rotation center
    Vec3f rot_center_;


    Mat4f model_;
    /// previous model matrix
    Mat4f old_model_matrix_;

public:
    /// constructor
    Trackball() :
        last_x_(0),
        last_y_(0),
        last_z_(0),
        start_point_3D_(0.0f),
        rot_center_(0.0f),
        model_(Mat4f::identity()),
        old_model_matrix_(Mat4f::identity()),
        radius_(1.0f),
        start_point_translation_(0.0f)
    {
    }

    /// initialize all values and matrices
    void init(const Vec3f& center, float radius) {
        rot_center_ = center;
        radius_ = radius;

        model_ = Mat4f::identity();
        old_model_matrix_ = Mat4f::identity();
    }

    void set_rotation_center(const Vec3f& c)
    {
        //model_ = Mat4f::identity();
        //old_model_matrix_ = Mat4f::identity();
        rot_center_ = c;
    }

    /// set new dimensions
    void set_dimensions(int width, int height)
    {
        width_ = width;
        height_ = height;
    }

    void start_translate(float x, float y, float z)
    {
        start_point_translation_[0] = x;
        start_point_translation_[1] = y;
        start_point_translation_[2] = z;

        last_x_ = x;
        last_y_ = y;
        last_z_ = z;
    }

    Vec3f translate(float x, float y, float z, const Vec3f& cam_pos)
    {
        if (!moved(x,y,z))
            return Vec3f(0.0f);

        //current point
        Vec3f p(x, y, z);
        //difference to previous point
        Vec3f d = p - start_point_translation_;


        //result: offset to translate
        Vec3f offset(
               -radius_ * 1.0f/width_  * d[0],
                radius_ * 1.0f/height_ * d[1],
                radius_ * 1.0f/height_ * d[2]);

        //scale with distance to rotation center
        float speed = norm(cam_pos-rot_center_);
        if (speed < 0.001f) speed = 0.001f;
        offset *= speed;

        //new start point
        start_point_translation_ = p;

        //return result
        return offset;
    }

    void start_rotation(int _x, int _y)
    {
        last_x_ = _x;
        last_y_ = _y;
        map_to_sphere(_x, _y, start_point_3D_);
    }

    /// rotates scene from movement in x and y
    /// \param[in] scale scale the "speed" of the operation
    void rotation(int x, int y, float scale = 1.0f)
    {
        if (!moved(x,y,0))
            return;

        Vec3f new_point_3D_;
        map_to_sphere(x, y, new_point_3D_);

        Vec3f axis      = cross(start_point_3D_, new_point_3D_);
        float cos_angle = dot(start_point_3D_, new_point_3D_);

        if (fabs(cos_angle) < 1.0f)
        {
            cos_angle = scale * 2.0f*acos(cos_angle) * 180.0f / PI_float;
            rotate(axis, cos_angle);
        }
    }

    const Mat4f& get_model() { return model_; }

private:
    /// maps x and y coordinates to sphere
    void map_to_sphere(int x_in, int y_in, Vec3f& p)
    {
        float x  = (float)(x_in - 0.5f*width_)  / (float)width_;
        float y  = (float)(0.5f*height_ - y_in) / (float)height_;
        float sinx         = sin(PI_float * x * 0.5f);
        float siny         = sin(PI_float * y * 0.5f);
        float sinx2siny2   = sinx * sinx + siny * siny;

        p[0] = sinx;
        p[1] = siny;
        p[2] = sinx2siny2 < 1.0f ? sqrtf(1.0f - sinx2siny2) : 0.0f;
    }


    /// calculate new matrices based on axis and angle
    void rotate (const Vec3f &axis, float angle)
    {
        model_ =
                Mat4f::translate(rot_center_) *
                Mat4f::rotate(axis, angle) *
                Mat4f::translate(-rot_center_) *
                old_model_matrix_;
        old_model_matrix_ = model_;
    }


    /// indicates if x and y differ from last x and y
    bool moved(int x, int y, int z)
    {
        return (last_x_ != x || last_y_ != y || last_z_ != z);
    }


};

} //namespace gl
} //namespace graphene
#endif // TRACKBALL_H
