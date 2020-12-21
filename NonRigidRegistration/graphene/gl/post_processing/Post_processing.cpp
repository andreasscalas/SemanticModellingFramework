#include <graphene/gl/post_processing/Post_processing.h>




namespace graphene
{
namespace gl
{

Post_processing::Post_processing() :
    write_buffer_(0),
    source_framebuffer_(NULL),
    linearz_framebuffer_(NULL)
{

}

Post_processing::~Post_processing()
{
    if (source_framebuffer_)
        delete source_framebuffer_;

    if (linearz_framebuffer_)
        delete linearz_framebuffer_;

    unsigned int i;
    for (i = 0; i < framebuffers_.size(); ++i)
    {
        if (framebuffers_[i])
            delete framebuffers_[i];
    }
}


void Post_processing::init(int width, int height)
{
    width_ = width;
    height_= height;

    source_framebuffer_ = new Framebuffer_multisample(width_, height_);

    quad_.init();

    while (framebuffers_.size() < 2)
    {
        framebuffers_.push_back(new Framebuffer_singlesample(width_, height_));
    }


}

void Post_processing::resize(int width, int height)
{
    if (width_ == width && height_ == height)
        return;

    width_ = width;
    height_ = height;

    if (source_framebuffer_)
        source_framebuffer_->resize(width_, height_);

    for (unsigned int i=0; i < framebuffers_.size(); ++i)
    {
        framebuffers_[i]->resize(width, height);
    }

    if (linearz_framebuffer_)
        linearz_framebuffer_->resize(width_, height_);
}

void Post_processing::bind()
{
    if (source_framebuffer_ == NULL)
        init(width_, height_);
    source_framebuffer_->bind();
}

void Post_processing::draw_quad()
{
    quad_.draw();
}

Framebuffer* Post_processing::get_source_fb()
{
    return source_framebuffer_;
}

Framebuffer* Post_processing::get_write_fb()
{
    return framebuffers_[write_buffer_];
}


Framebuffer* Post_processing::get_read_fb()
{
    return framebuffers_[1-write_buffer_];
}

Framebuffer* Post_processing::get_linear_z_fb()
{
    if (linearz_framebuffer_ == NULL)
        linearz_framebuffer_ = new Framebuffer_singlesample(width_,height_);

    return linearz_framebuffer_;
}

Framebuffer* Post_processing::get_fb(unsigned int idx)
{
    Framebuffer* fb;
    while (idx >= framebuffers_.size())
    {
        fb = new Framebuffer_singlesample(width_, height_);
        framebuffers_.push_back(fb);
    }
    fb = framebuffers_[idx];
    return fb;
}

void Post_processing::swap_buffers()
{
    write_buffer_ = 1 - write_buffer_;
}

} //namespace gl
} //namespace graphene
