
#include <graphene/gl/post_processing/Post_process_list.h>

namespace graphene
{
namespace gl
{



Post_process_list::Post_process_list() :
    initialized_(false),
    pp_heap_(post_processing_hierarchy::MAX, NULL)
{}

Post_process_list::~Post_process_list()
{
    Post_process* pp;
    for (unsigned int i=0; i < pp_heap_.size(); ++i)
    {
        pp = pp_heap_[i];
        if (pp)
            delete pp;
    }
}

void Post_process_list::init()
{
    if (initialized_)
        return;

    initialized_ = true;

    pp_heap_[post_processing_hierarchy::AMBIENTOCCLUSION] = new PP_ambientocclusion;
    pp_heap_[post_processing_hierarchy::BLUR] = new PP_blur;
    pp_heap_[post_processing_hierarchy::FINALIZE] = new PP_finalize;
    pp_heap_[post_processing_hierarchy::LINEARIZE_Z] = new PP_linearize_z;
    pp_heap_[post_processing_hierarchy::MIX] = new PP_mix;
    pp_heap_[post_processing_hierarchy::SUBSURFACE_SCATTERING] = new PP_subsurface_scattering;
    pp_heap_[post_processing_hierarchy::SHADOW_MAPPING] = new PP_shadow_mapping;


    pp_names_.push_back(pp_heap_[post_processing_hierarchy::AMBIENTOCCLUSION]->name_);
    pp_names_.push_back(pp_heap_[post_processing_hierarchy::SHADOW_MAPPING]->name_);
    pp_names_.push_back(pp_heap_[post_processing_hierarchy::SUBSURFACE_SCATTERING]->name_);
}

void Post_process_list::print()
{
    std::list<Post_process*>::iterator it, it_end = pp_list_.end();
    Post_process* pp;
    int i = 0;
    for (it = pp_list_.begin(); it != it_end; ++it)
    {
        pp = *it;
        std::cout << ++i << ": ";
        std::cout << pp->name_ << std::endl;
    }
}

void Post_process_list::clear()
{
    pp_list_.clear();

    Post_process* pp;
    for (unsigned int i=0; i < pp_heap_.size(); ++i)
    {
        pp = pp_heap_[i];
        if (pp)
        {
            pp->active_ = false;
        }
    }
}

void Post_process_list::apply(GL_state *gls)
{
    std::list<Post_process*>::iterator it, it_end = pp_list_.end();

    Framebuffer* fb;
    Vec4i viewport;

    glGetIntegerv(GL_VIEWPORT, viewport.data());

    glDisable(GL_MULTISAMPLE);
    glDisable(GL_DEPTH_TEST);

    //resolve multisampling
    fb = gls->post_processing_.get_fb(2);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fb->get_fbo_id());
    fb = gls->post_processing_.get_source_fb();
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fb->get_fbo_id());

    glBlitFramebuffer(viewport[0],viewport[1],viewport[2],viewport[3],
                      viewport[0],viewport[1],viewport[2],viewport[3],
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);

    //clear first read buffer to have a clean start
    gls->post_processing_.get_read_fb()->bind();
    glClearBufferfv(GL_COLOR, fragdata_locations::COLOR, Vec4f(1.0f).data());

    Post_process* pp;
    for (it = pp_list_.begin(); it != it_end; ++it)
    {
        pp = *it;
        pp->apply(gls);
    }

    if (gls->multisampling_enabled_)
        glEnable(GL_MULTISAMPLE);

    glEnable(GL_DEPTH_TEST);
}

bool Post_process_list::is_empty()
{
    return pp_list_.empty();
}

bool Post_process_list::is_shadowmapping_active()
{
    return pp_heap_[post_processing_hierarchy::SHADOW_MAPPING]->active_;
}


const std::vector<std::string> &Post_process_list::get_post_processes()
{
    return pp_names_;
}

void Post_process_list::select(const std::string& name)
{
    unsigned int i,j;
    Post_process *pp;
    for (i=0; i < pp_heap_.size(); ++i)
    {
        pp = pp_heap_[i];
        if (pp)
        {
            if (pp->name_.compare(name) == 0)
            {
                for (j=0; j < pp->needed_pps_.size(); ++j)
                {
                    pp_heap_[pp->needed_pps_[j]]->active_ = true;
                }
            }
        }
    }
}

void Post_process_list::create()
{
    pp_list_.clear();

    unsigned int i;
    Post_process* pp;
    for (i=0; i < pp_heap_.size(); ++i)
    {
        pp = pp_heap_[i];
        if (pp)
        {
            if (pp->active_)
            {
                pp_list_.push_back(pp);
            }
        }
    }
    if (!pp_list_.empty())
    {
        pp_list_.push_back(pp_heap_[post_processing_hierarchy::FINALIZE]);
    }
}

} //namespace gl
} //namespace graphene
