 
#ifndef GRAPHENE_POST_PROCESSING_H
#define GRAPHENE_POST_PROCESSING_H

#include <graphene/gl/Framebuffer.h>
#include <graphene/gl/post_processing/Quad.h>

namespace graphene
{
namespace gl
{

class Post_processing
{
    int width_;
    int height_;

    int write_buffer_;

    Framebuffer* source_framebuffer_;

    Framebuffer* linearz_framebuffer_;

    std::vector<Framebuffer*> framebuffers_;

    Quad quad_;
public:
    Post_processing();
    ~Post_processing();

    void init(int width = 8, int height = 8);

    void resize(int width, int height);

    void bind();

    void draw_quad();

    Framebuffer* get_source_fb();
    Framebuffer* get_write_fb();
    Framebuffer* get_read_fb();
    Framebuffer* get_linear_z_fb();
    Framebuffer* get_fb(unsigned int idx);

    void swap_buffers();
};

} //namespace gl
} //namespace graphene

#endif
