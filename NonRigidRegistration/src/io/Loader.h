#ifndef LOADER_H
#define LOADER_H

#include <string>
#include <vector>

#include <graphene/scene_graph/Scene_graph.h>
#include <graphene/character/scene_graph/Character_node.h>
#include <graphene/scene_graph/Point_set_node.h>

using namespace graphene;

namespace io
{
class Loader
{
private:
    gl::GL_state& gl_state_;
    scene_graph::Scene_graph& scene_graph_;

public:
    Loader(gl::GL_state& gl_state, scene_graph::Scene_graph& scene_graph);

    ~Loader();

    bool load(const std::vector<std::string>& filenames);
    bool load(const std::string& filename);

private:

    bool load_textures(const surface_mesh::Surface_mesh& mesh);
};

} // namespace io


#endif
