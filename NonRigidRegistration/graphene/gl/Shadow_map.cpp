#include <graphene/gl/Shadow_map.h>


namespace graphene
{
namespace gl
{

Shadow_map::Shadow_map(int width, int height)
{
    width_ = width;
    height_ = height;

    glGenFramebuffers(1, &framebuffer_id_);
    glGenTextures(1, &depth_texture_);
    glGenTextures(1, &color_texture_);

    glActiveTexture(GL_TEXTURE0);

    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float border_color[] = {1.0f,1.0f,1.0f,1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_GREATER);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width_, height_, 0, GL_RGBA, GL_FLOAT, 0);


    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_id_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);

    GLenum attachment = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &attachment);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "Shadowmap-Framebuffer incomplete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

Shadow_map::~Shadow_map()
{
    glDeleteFramebuffers(1, &framebuffer_id_);
    glDeleteTextures(1, &depth_texture_);
    glDeleteTextures(1, &color_texture_);
}

void Shadow_map::setup(const Vec3f &center, const Vec3f &direction, float radius)
{
    Vec3f position = center + (2.0f*radius * normalize(direction));

    //float near = postion[2] - radius;
    //float far = near + 2.0f * radius;

    view_ = Mat4f::look_at(position, center, Vec3f(0.0f,1.0f,0.0f));

    near_ = radius;
    far_  = 5.0f*radius;

    radius *= 0.75f;

    proj_ = Mat4f::frustum(-radius, radius, -radius, radius, near_, far_);
            //Mat4f::perspective(fovy_, (float) width_/(float) height_, near_, far_);

    viewproj_ = proj_ * view_;
}


void Shadow_map::activate()
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_id_);
    //backup viewport
    glGetIntegerv(GL_VIEWPORT, viewport_backup_.data());
    //set viewport
    glViewport(0,0, width_,height_);

    glClear(GL_DEPTH_BUFFER_BIT);
}

void Shadow_map::deactivate()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //restore viewport
    glViewport(viewport_backup_[0], viewport_backup_[1], viewport_backup_[2], viewport_backup_[3]);
}

GLuint Shadow_map::get_depth_texture_id()
{
    return depth_texture_;
}


} //namespace gl
} //namespace graphene
