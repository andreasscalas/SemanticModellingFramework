#ifndef GRAPHENE_SHADOW_MAP_H
#define GRAPHENE_SHADOW_MAP_H

#include <graphene/gl/gl_includes.h>
#include <graphene/geometry/Matrix4x4.h>

namespace graphene
{
namespace gl
{

class Shadow_map
{
private:
    int width_;
    int height_;


    Vec4i viewport_backup_;

    GLuint framebuffer_id_;

    GLuint depth_texture_;
    GLuint color_texture_;

public:
    Mat4f viewproj_;
    Mat4f proj_;
    Mat4f view_;
    float near_;
    float far_;

public:
    Shadow_map(int width, int height);
    ~Shadow_map();

    void setup(const Vec3f& center, const Vec3f& position,float radius);

    void activate();
    void deactivate();

    GLuint get_depth_texture_id();

};

} //namespace gl
} //namespace graphene

#endif
