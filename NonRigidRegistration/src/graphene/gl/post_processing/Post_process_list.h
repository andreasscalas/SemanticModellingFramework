 
#ifndef GRAPHENE_POST_PROCESS_LIST_H
#define GRAPHENE_POST_PROCESS_LIST_H

#include <graphene/gl/post_processing/Post_process.h>
#include <graphene/gl/post_processing/Post_processes.h>

#include <list>
#include <map>
namespace graphene
{
namespace gl
{




class Post_process_list
{
    bool initialized_;

    std::vector<Post_process*> pp_heap_;

    std::list<Post_process*> pp_list_;

    std::vector<std::string> pp_names_;

public:

    Post_process_list();
    ~Post_process_list();

    ///call this AFTER initialization of GL/GLEW
    void init();

    void print();

    void clear();

    void apply(GL_state* gls);

    bool is_empty();

    bool is_shadowmapping_active();

    const std::vector<std::string>& get_post_processes();

    void select(const std::string &name);

    void create();
};

} //namespace gl
} //namespace graphene

#endif
