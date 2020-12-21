#include "Framebuffer.h"


namespace graphene
{
namespace gl
{

//=SINGLESAMPLE================================================================


Framebuffer_singlesample::Framebuffer_singlesample(int width, int height, bool with_depth,
                                                   GLint internal_format_color, GLint internal_format_depth) :
    Framebuffer(width, height, with_depth, internal_format_color, internal_format_depth)
{
    glGenFramebuffers(1, &fbo_id_);

    //set one default color attachment
    set_color_attachment(fragdata_locations::COLOR, internal_format_color);

    //set default depth component
    if (with_depth)
    {
        set_depth_attachment(internal_format_depth);
    }



    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "Framebuffer incomplete!" << std::endl;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


}

Framebuffer_singlesample::~Framebuffer_singlesample()
{
    if (depth_attachment_)
        delete depth_attachment_;

    Attachment* a = 0;
    for (unsigned int i=0; i < color_attachments_.size(); ++i)
    {
        a = color_attachments_[i];
        if (a)
        {
            delete a;
        }
    }

    glDeleteFramebuffers(1, &fbo_id_);
}


void Framebuffer_singlesample::bind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
}

void Framebuffer_singlesample::resize(int width, int height)
{
    width_ = width;
    height_ = height;

    Attachment* a;
    for (unsigned int i=0; i < color_attachments_.size(); ++i)
    {
        a = color_attachments_[i];
        if (a)
        {
            glBindTexture(GL_TEXTURE_2D, a->texture_id_);
            glTexImage2D(GL_TEXTURE_2D, 0, a->internal_format_, width_, height_, 0, GL_RGBA, GL_FLOAT, 0);
        }
    }


    glBindTexture(GL_TEXTURE_2D, depth_attachment_->texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, depth_attachment_->internal_format_,
                 width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
}


void Framebuffer_singlesample::set_color_attachment(int location, GLint internal_format)
{
    Attachment* a = color_attachments_[location];
    //already up and running => just set texture values
    if (a)
    {
        a->internal_format_ = internal_format;
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, a->internal_format_, width_, height_, 0, GL_RGBA, GL_FLOAT, 0);
    }
    //not active => create new and add to drawbuffers and framebuffer
    else
    {
        a = new Attachment(GL_COLOR_ATTACHMENT0 + location,
                           internal_format);

        color_attachments_[location] = a;

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, a->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, a->internal_format_, width_, height_, 0, GL_RGBA, GL_FLOAT, 0);


        glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
        //bind texture to framebuffer
        glFramebufferTexture(GL_FRAMEBUFFER, a->attachment_id_, a->texture_id_, 0);

        //collect active attachments
        std::vector<GLenum> attachments;
        for (unsigned int i=0; i < color_attachments_.size(); ++i)
        {
            a = color_attachments_[i];
            if (a) {
                attachments.push_back(a->attachment_id_);
            }
        }

        glDrawBuffers(attachments.size(), &attachments[0]);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void Framebuffer_singlesample::set_depth_attachment(GLint format)
{
    if (depth_attachment_)
    {
        depth_attachment_->internal_format_ = format;
        glBindTexture(GL_TEXTURE_2D, depth_attachment_->texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, depth_attachment_->internal_format_, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    }
    else
    {
        glActiveTexture(GL_TEXTURE0);

        depth_attachment_ = new Attachment(-1,//depth does not have fragdata location
                           format);
        glBindTexture(GL_TEXTURE_2D, depth_attachment_->texture_id_);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, depth_attachment_->internal_format_, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

        //bind texture to framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_attachment_->texture_id_, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

}


Attachment*
Framebuffer_singlesample::
get_color_attachment()
{
    return color_attachments_[fragdata_locations::COLOR];
}


void Framebuffer_singlesample::bind_color_texture(int attachment, GLuint shader_id, const char *samplername, int texture_unit)
{
    Attachment* at = color_attachments_[attachment];
    if (at == NULL)
        return;

    glActiveTexture(GL_TEXTURE0 + texture_unit);
    glBindTexture(GL_TEXTURE_2D, at->texture_id_);

    GLint id = glGetUniformLocation(shader_id, samplername);
    if (id == -1) std::cerr << "Framebuffer: Invalid texture name \"" << samplername <<"\"!" << std::endl;
    glUniform1i(id, texture_unit);

    glActiveTexture(GL_TEXTURE0);
}

void Framebuffer_singlesample::bind_depth_texture(GLuint shader_id, const char *samplername, int texture_unit)
{
    Attachment* at = depth_attachment_;
    if (at == NULL)
        return;

    glActiveTexture(GL_TEXTURE0 + texture_unit);
    glBindTexture(GL_TEXTURE_2D, at->texture_id_);

    GLint id = glGetUniformLocation(shader_id, samplername);
    if (id == -1) std::cerr << "Framebuffer: Invalid texture name \"" << samplername <<"\"!" << std::endl;
    glUniform1i(id, texture_unit);

    glActiveTexture(GL_TEXTURE0);
}

//=SINGLESAMPLE_END============================================================



//=MULTISAMPLE=================================================================

Framebuffer_multisample::Framebuffer_multisample(int width, int height, bool with_depth,
                                                 GLint internal_format_color, GLint internal_format_depth) :
    Framebuffer(width, height, with_depth, internal_format_color, internal_format_depth),
    multisamples_(1)
{
    glGenFramebuffers(1, &fbo_id_);

    glGetIntegerv(GL_SAMPLES, &multisamples_);
    if (multisamples_ == 0)
        multisamples_ = 1;

    //set one default color attachment
    set_color_attachment(fragdata_locations::COLOR, internal_format_color);

    //set default depth component
    if (with_depth)
    {
        set_depth_attachment(internal_format_depth);
    }



    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);


    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << "Framebuffer incomplete!" << std::endl;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


}

Framebuffer_multisample::~Framebuffer_multisample()
{
    if (depth_attachment_)
        delete depth_attachment_;

    Attachment* a = 0;
    for (unsigned int i=0; i < color_attachments_.size(); ++i)
    {
        a = color_attachments_[i];
        if (a)
        {
            delete a;
        }
    }

    glDeleteFramebuffers(1, &fbo_id_);
}


void Framebuffer_multisample::bind()
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
}

void Framebuffer_multisample::resize(int width, int height)
{
    width_ = width;
    height_ = height;

    Attachment* a;
    for (unsigned int i=0; i < color_attachments_.size(); ++i)
    {
        a = color_attachments_[i];
        if (a)
        {
            glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, a->texture_id_);
            glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples_, a->internal_format_, width_, height_, GL_FALSE);
        }
    }


    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, depth_attachment_->texture_id_);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples_, depth_attachment_->internal_format_,
                 width_, height_, GL_FALSE);
}

void Framebuffer_multisample::set_color_attachment(int location, GLint internal_format)
{
    Attachment* a = color_attachments_[location];
    //already up and running => just set texture values
    if (a)
    {
        a->internal_format_ = internal_format;
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, a->texture_id_);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples_, a->internal_format_, width_, height_, GL_FALSE);
    }
    //not active => create new and add to drawbuffers and framebuffer
    else
    {
        a = new Attachment(GL_COLOR_ATTACHMENT0 + location,
                           internal_format);

        color_attachments_[location] = a;

        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, a->texture_id_);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples_, a->internal_format_, width_, height_, GL_FALSE);


        glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
        //bind texture to framebuffer
        glFramebufferTexture(GL_FRAMEBUFFER, a->attachment_id_, a->texture_id_, 0);

        //collect active attachments
        std::vector<GLenum> attachments;
        for (unsigned int i=0; i < color_attachments_.size(); ++i)
        {
            a = color_attachments_[i];
            if (a) {
                attachments.push_back(a->attachment_id_);
            }
        }

        glDrawBuffers(attachments.size(), &attachments[0]);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
}

void Framebuffer_multisample::set_depth_attachment(GLint format)
{
    if (depth_attachment_)
    {
        depth_attachment_->internal_format_ = format;
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, depth_attachment_->texture_id_);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples_, depth_attachment_->internal_format_, width_, height_, GL_FALSE);
    }
    else
    {

        depth_attachment_ = new Attachment(-1,//depth does not have fragdata location
                           format);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, depth_attachment_->texture_id_);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamples_, depth_attachment_->internal_format_, width_, height_, GL_FALSE);

        //bind texture to framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_attachment_->texture_id_, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

}


Attachment*
Framebuffer_multisample::
get_color_attachment()
{
     return color_attachments_[fragdata_locations::COLOR];
}


void Framebuffer_multisample::bind_color_texture(int attachment, GLuint shader_id, const char *samplername, int texture_unit)
{
    Attachment* at = color_attachments_[attachment];
    if (at == NULL)
        return;

    glActiveTexture(GL_TEXTURE0 + texture_unit);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, at->texture_id_);

    GLint id = glGetUniformLocation(shader_id, samplername);
    if (id == -1) std::cerr << "Framebuffer: Invalid texture name \"" << samplername <<"\"!" << std::endl;
    glUniform1i(id, texture_unit);

    glActiveTexture(GL_TEXTURE0);
}

void Framebuffer_multisample::bind_depth_texture(GLuint shader_id, const char *samplername, int texture_unit)
{
    Attachment* at = depth_attachment_;
    if (at == NULL)
        return;

    glActiveTexture(GL_TEXTURE0 + texture_unit);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, at->texture_id_);

    GLint id = glGetUniformLocation(shader_id, samplername);
    if (id == -1) std::cerr << "Framebuffer: Invalid texture name \"" << samplername <<"\"!" << std::endl;
    glUniform1i(id, texture_unit);

    glActiveTexture(GL_TEXTURE0);
}


//=MULTISAMPLE_END=============================================================

} //namespace gl
} //namespace graphene
