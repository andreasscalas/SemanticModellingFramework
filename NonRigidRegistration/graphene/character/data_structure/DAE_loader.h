#ifndef GRAPHENE_DAE_LOADER_H
#define GRAPHENE_DAE_LOADER_H

#include <tinyxml/tinyxml.h>
#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Matrix4x4.h>

#include <map>
#include <vector>

namespace graphene
{
namespace character
{



class DAE_loader
{
public:

    //helper structs

    struct DAE_Effect
    {
        std::string id_;

        std::string texture_filename_;
        std::string texture_filename_specmap_;
        std::string texture_filename_normalmap_;
        Vec2f texture_repeat_;


        Vec3f emission_color_;
        Vec3f ambient_color_;
        Vec3f diffuse_color_;
        Vec4f specular_color_;

        DAE_Effect() :
            texture_repeat_(1.0f),
            emission_color_(0.0f),
            ambient_color_(0.0f),
            diffuse_color_(0.0f),
            specular_color_(0.0f)
        {}
    };

    struct DAE_Mesh
    {
        std::string id_;

        std::vector<unsigned int> indices_;

        std::vector<float> vertices_;
        int vertex_stride_;
        int vertex_offset_;
        std::vector<float> normals_;
        int normal_stride_;
        int normal_offset_;
        std::vector<float> colors_;
        int color_stride_;
        int color_offset_;
        std::vector<float> texcoords_;
        int texcoord_stride_;
        int texcoord_offset_;

        int index_step_;

        DAE_Effect* effect_;

        std::vector<Mat4f> instances_;

        DAE_Mesh() :
            effect_(NULL),
            vertex_offset_(-1), vertex_stride_(1),
            normal_offset_(-1), normal_stride_(1),
            color_offset_(-1), color_stride_(1),
            texcoord_offset_(-1), texcoord_stride_(1),
            index_step_(1)
        {}
    };

    struct DAE_Light
    {
        std::string id_;

        Vec3f color_;
        Vec3f attenuation_;

        std::vector<Vec3f> positions_;

        DAE_Light() :
            color_(1.0f),
            attenuation_(1.0f,0.0f,0.0f)
        {}
    };

    struct DAE_Morph
    {
        std::string id_;

        std::string base_mesh_id_;

        std::vector<std::string> target_ids_;
        std::vector<float> morph_weights_;

        std::vector<Mat4f> instances_;
        DAE_Effect* effect_;

        DAE_Morph() :
            effect_(NULL)
        {}
    };

    struct DAE_Skin
    {
        std::string id_;

        std::string target_id_;

        std::vector<int> vcount_;
        std::vector<int> v_;

        std::vector<float> weights_array_;
        int weights_offset_;

        std::vector<std::string> joints_array_;
        int joints_offset_;

        int index_step_;

        std::vector<Mat4f> instances_;
        DAE_Effect* effect_;

        DAE_Skin() :
            weights_offset_(-1),
            joints_offset_(-1),
            index_step_(1),
            effect_(NULL)
        {}

    };

    struct DAE_Joint
    {
        std::string name_;
        DAE_Joint* parent_;
        std::vector<DAE_Joint*> children_;
        Mat4f transform_;

        DAE_Joint() :
            parent_(NULL)
        {}
    };

    struct DAE_Skeleton
    {
        DAE_Joint* root_;
        std::vector<DAE_Joint*> joints_;

        DAE_Skeleton() :
            root_(NULL)
        {}

        ~DAE_Skeleton()
        {
            for (size_t i=0; i < joints_.size(); ++i)
                delete joints_[i];
        }
    };

public:

    std::map<std::string, std::string> texture_lib_;
    std::map<std::string, DAE_Effect*> effect_lib_;
    std::map<std::string, DAE_Mesh*> mesh_lib_;
    std::map<std::string, DAE_Light> light_lib_;
    std::map<std::string, DAE_Morph*> morph_lib_;
    std::map<std::string, DAE_Skin*> skin_lib_;

    std::vector<DAE_Skeleton*> skeletons_;

    float global_scale_;
    Mat4f global_transform_;

    TiXmlDocument xml_;

    std::string filename_;

public:
    DAE_loader();

    ~DAE_loader();

    bool load(const std::string &filename);
private:

    bool read_root_asset(TiXmlElement* e);

    bool read_lights_lib(TiXmlElement* e);
    bool read_light(TiXmlElement* e);

    bool read_images_lib(TiXmlElement* e);
    bool read_image(TiXmlElement* e);

    bool read_materials_lib(TiXmlElement* e);

    bool read_effects_lib(TiXmlElement* e);
    bool read_effect(TiXmlElement* e, DAE_Effect* effect);
    bool read_texture(TiXmlElement* texture_e, TiXmlElement *base_e, DAE_Effect* effect, std::string& texture_name);

    bool read_geometries_lib(TiXmlElement* e);
    bool read_geometry(TiXmlElement* e);
    bool read_mesh(TiXmlElement* e, DAE_Mesh *mesh);

    bool read_controllers_lib(TiXmlElement* e);
    bool read_controller(TiXmlElement* e);
    bool read_morph(TiXmlElement* e, DAE_Morph *morph);
    bool read_skin(TiXmlElement* e, DAE_Skin *skin);

    bool read_visual_scenes_lib(TiXmlElement* e);
    bool read_visual_scene(TiXmlElement* e);
    void read_visual_scene_node_recursive(TiXmlElement* e, Mat4f &transform);
    void read_instance_geometry(TiXmlElement* e, const Mat4f& transform);
    void read_instance_light(TiXmlElement* e, const Mat4f& transform);
    void read_instance_controller(TiXmlElement* e, const Mat4f& transform);
    void read_joint_node_recursive(TiXmlElement* e, DAE_Joint *parent, DAE_Skeleton* skeleton);


    bool read_source(TiXmlElement *e, int *count, int *stride, std::vector<float> &float_array);
    bool read_source(TiXmlElement *e, int *count, int *stride, std::vector<std::string> &string_array);


    //helper
    std::map<std::string, TiXmlElement*> visual_scene_node_lib_;
    std::map<std::string, DAE_Effect*> materialid_to_effect_;

    void build_node_lib_recursive(TiXmlElement* e);
    const Mat4f get_local_transform(TiXmlElement* e);

    TiXmlElement* get_sibling_by_name(TiXmlElement* e, const std::string &name);
    TiXmlElement* get_sibling_by_attr_value(TiXmlElement* e, const std::string &attr, const std::string &val);
    TiXmlElement* get_element_by_name_recursive(TiXmlElement* e, const std::string &name);

};


} //namespace character
} //namespace graphene

#endif
