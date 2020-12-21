//=============================================================================

#ifndef GRAPHENE_CHARACTER_H
#define GRAPHENE_CHARACTER_H

//=============================================================================

#include <graphene/character/data_structure/Skeleton.h>
#include <graphene/character/data_structure/Surface_mesh_skin.h>
#include <graphene/character/data_structure/Blendshapes.h>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


class Character
{
private:
    std::string name_;

    Skeleton skeleton_;

    std::vector<Surface_mesh_skin*> skins_;
    int selected_skin_idx_;

    std::vector<Blendshapes*> blendshapes_;

public:

    Character();
    ~Character();

    const geometry::Bounding_box bbox();

    bool read(const char* filename);

    bool write_bim(const char* filename) const;
    bool write_fbx(const char* filename) const;

    void delete_skin(Surface_mesh_skin* skin);

    void apply_skinning_on_CPU( bool only_selected_skin = false, Skinning_mode mode = LINEAR_BLENDING);
    void apply_skinning_on_CPU(const std::vector<surface_mesh::Surface_mesh::Vertex>& indices, std::vector<Point>& result, Skinning_mode mode = LINEAR_BLENDING);

    void set_current_pose_as_bindpose();

    void correct_joint_positions();

    void set_new_joint_positions(const std::vector<Vec3f>& positions);

    bool delete_joint(const std::string& joint_name);

public:
    //getter/setter
    void set_name(const std::string& name) {name_ = name;}
    const std::string& get_name() const { return name_; }
    Skeleton& skeleton() { return skeleton_; }
    const Skeleton& skeleton() const { return skeleton_; }
    std::vector<Surface_mesh_skin*>& skins() { return skins_; }
    const std::vector<Surface_mesh_skin*>& skins() const { return skins_; }
    std::vector<Blendshapes*>& blendshapes() { return blendshapes_; }
    const std::vector<Blendshapes*>& blendshapes() const { return blendshapes_; }
    Surface_mesh_skin* get_selected_skin() {return skins_[selected_skin_idx_];}
    const Surface_mesh_skin* get_selected_skin() const {return skins_[selected_skin_idx_];}
    int get_selected_skin_idx() {return selected_skin_idx_;}
    void set_selected_skin_idx(int idx) { selected_skin_idx_ = idx; }
    int get_skin_idx(const std::string& name);



private:
    void apply_linear_skinning_on_CPU();
    void apply_dualquat_skinning_on_CPU();

    void apply_linear_skinning_on_CPU_selected_skin();
    void apply_dualquat_skinning_on_CPU_selected_skin();

    void apply_linear_skinning_on_CPU(const unsigned int skin_idx, const std::vector<surface_mesh::Surface_mesh::Vertex> &indices, std::vector<Point>& result);
    void apply_dualquat_skinning_on_CPU(const unsigned int skin_idx, const std::vector<surface_mesh::Surface_mesh::Vertex> &indices, std::vector<Point>& result);

    //Reads the skeleton information from a .model file
    bool read_model_character(Character *character, const char *filename);

};


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

