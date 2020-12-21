#ifndef GRAPHENE_FRAMEBUFFER_GL_H
#define GRAPHENE_FRAMEBUFFER_GL_H

#include <graphene/gl/gl_includes.h>
#include <graphene/gl/Shader.h>
#include <vector>

namespace graphene
{
namespace gl
{


struct Attachment
{
    GLenum attachment_id_;

    GLint internal_format_;

    GLuint texture_id_;

    Attachment(GLenum id, GLint internal_format) :
        attachment_id_(id), internal_format_(internal_format), texture_id_(0)
    {
        glGenTextures(1, &texture_id_);
    }

    ~Attachment()
    {
        if (texture_id_) glDeleteTextures(1, &texture_id_);
    }
};

class Framebuffer
{
protected:
    GLuint fbo_id_;

    Attachment* depth_attachment_;

    std::vector<Attachment*> color_attachments_;

    int width_;
    int height_;

public:
    Framebuffer(int width, int height, bool with_depth = true,
                GLint internal_format_color = GL_RGBA16, GLint internal_format_depth = GL_DEPTH_COMPONENT24) :
        fbo_id_(0), depth_attachment_(NULL), color_attachments_(fragdata_locations::MAX, 0),
        width_(width), height_(height)
    {  }

    virtual ~Framebuffer() {}
    
    virtual void bind() = 0;

    virtual void resize(int width, int height) = 0;

    virtual bool has_color_attachment(int location) { return color_attachments_[location] != NULL; }

    ///if the attachment at "location" is already active this just changes the texture values
    ///else it adds a new attachment
    virtual void set_color_attachment(int location, GLint internal_format) = 0;

    virtual void set_depth_attachment(GLint format) = 0;

    virtual Attachment* get_color_attachment() = 0;

    virtual void bind_color_texture(int attachment, GLuint shader_id, const char* samplername, int texture_unit) = 0;
    virtual void bind_depth_texture(GLuint shader_id, const char* samplername, int texture_unit) = 0;

    virtual void swap_attachments(int location0, int location1)
    {
        Attachment *a0,*a1;
        a0 = color_attachments_[location0];
        a1 = color_attachments_[location1];
        GLuint tmp_texid = a0->texture_id_;
        a0->texture_id_ = a1->texture_id_;
        a1->texture_id_ = tmp_texid;

        glFramebufferTexture(GL_FRAMEBUFFER, a0->attachment_id_, a0->texture_id_, 0);
        glFramebufferTexture(GL_FRAMEBUFFER, a1->attachment_id_, a1->texture_id_, 0);
    }

    GLuint get_fbo_id()
    {
        return fbo_id_;
    }
};


class Framebuffer_singlesample : public Framebuffer
{
private:

public:
    Framebuffer_singlesample(int width, int height, bool with_depth = true,
                             GLint internal_format_color = GL_RGBA16, GLint internal_format_depth = GL_DEPTH_COMPONENT24);

    ~Framebuffer_singlesample();

    void bind();

    void resize(int width, int height);

    void set_color_attachment(int location, GLint internal_format);

    void set_depth_attachment(GLint format);

    Attachment* get_color_attachment();

    void bind_color_texture(int attachment, GLuint shader_id, const char* samplername, int texture_unit);

    void bind_depth_texture(GLuint shader_id, const char* samplername, int texture_unit);
};

class Framebuffer_multisample : public Framebuffer
{
private:
    int multisamples_;

public:
    Framebuffer_multisample(int width, int height, bool with_depth = true,
                            GLint internal_format_color = GL_RGBA16, GLint internal_format_depth = GL_DEPTH_COMPONENT24);

    ~Framebuffer_multisample();

    void bind();

    void resize(int width, int height);

    void set_color_attachment(int location, GLint internal_format);

    void set_depth_attachment(GLint format);

    Attachment* get_color_attachment();

    void bind_color_texture(int attachment, GLuint shader_id, const char* samplername, int texture_unit);

    void bind_depth_texture(GLuint shader_id, const char* samplername, int texture_unit);
};

} // namespace gl
} // namespace graphene

#endif //GRAPHENE_FRAMEBUFFER_GL_H
