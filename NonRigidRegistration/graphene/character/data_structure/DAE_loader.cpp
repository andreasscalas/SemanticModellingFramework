#include "DAE_loader.h"

#include <cstring>



namespace graphene
{
namespace character
{

DAE_loader::DAE_loader() :
    global_scale_(1.0f),
    global_transform_(Mat4f::identity())
{
}

DAE_loader::~DAE_loader()
{
    std::map<std::string, DAE_Mesh*>::iterator mesh_it;
    for (mesh_it = mesh_lib_.begin(); mesh_it != mesh_lib_.end(); ++mesh_it)
    {
        delete mesh_it->second;
    }

    std::map<std::string, DAE_Effect*>::iterator effect_it;
    for (effect_it = effect_lib_.begin(); effect_it != effect_lib_.end(); ++effect_it)
    {
        delete effect_it->second;
    }

    std::map<std::string, DAE_Morph*>::iterator morph_it;
    for (morph_it = morph_lib_.begin(); morph_it != morph_lib_.end(); ++morph_it)
    {
        delete morph_it->second;
    }

    std::map<std::string, DAE_Skin*>::iterator skin_it;
    for (skin_it = skin_lib_.begin(); skin_it != skin_lib_.end(); ++skin_it)
    {
        delete skin_it->second;
    }

    for (size_t i=0; i < skeletons_.size(); ++i)
    {
        delete skeletons_[i];
    }
}


bool DAE_loader::load(const std::string& filename)
{
    if (! xml_.LoadFile(filename))
            return false;

    if (xml_.RootElement()->ValueStr() != "COLLADA")
        return false;

    TiXmlElement* elem = xml_.RootElement()->FirstChildElement();
    TiXmlElement
            *root_asset=NULL,
            *lights_lib=NULL,
            *images_lib=NULL,
            *materials_lib=NULL,
            *effects_lib=NULL,
            *geometries_lib=NULL,
            *visual_scenes_lib=NULL,
            *controller_lib=NULL;

    filename_ = filename;

    while (elem)
    {
        if (elem->ValueStr() == "asset")
            root_asset = elem;
        else if (elem->ValueStr() == "library_lights")
            lights_lib = elem;
        else if (elem->ValueStr() == "library_images")
            images_lib = elem;
        else if (elem->ValueStr() == "library_materials")
            materials_lib = elem;
        else if (elem->ValueStr() == "library_effects")
            effects_lib = elem;
        else if (elem->ValueStr() == "library_geometries")
            geometries_lib = elem;
        else if (elem->ValueStr() == "library_visual_scenes")
            visual_scenes_lib = elem;
        else if (elem->ValueStr() == "library_controllers")
                controller_lib = elem;

        elem = elem->NextSiblingElement();
    }

    if (root_asset)
        read_root_asset(root_asset);

    if (lights_lib)
        read_lights_lib(lights_lib);

    if (images_lib)
        read_images_lib(images_lib);

    if (effects_lib)
        read_effects_lib(effects_lib);

    if (materials_lib)
        read_materials_lib(materials_lib);


    if (geometries_lib)
        read_geometries_lib(geometries_lib);

    if (controller_lib)
        read_controllers_lib(controller_lib);

    if (visual_scenes_lib)
        read_visual_scenes_lib(visual_scenes_lib);



    return true;
}

bool DAE_loader::read_root_asset(TiXmlElement *e)
{
    Mat4f rotate(0.0f);
    std::string up_axis;
    std::stringstream ss;

    e = e ->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "unit")
        {
            if (e->Attribute("meter"))
            {
                ss.clear();
                ss.str(e->Attribute("meter"));

                ss >> global_scale_;
            }
        }
        else if (e->ValueStr() == "up_axis")
        {
            up_axis = e->GetText();
        }

        e = e->NextSiblingElement();
    }

    if (up_axis == "X_UP")
    {
        rotate(0,1) = -1.0f;
        rotate(1,0) =  1.0f;
        rotate(2,2) =  1.0f;
        rotate(3,3) =  1.0f;
    }
    else if (up_axis == "Y_UP")
    {
        rotate = Mat4f::identity();
    }
    else if (up_axis == "Z_UP")
    {
        rotate(0,0) =  1.0f;
        rotate(1,2) =  1.0f;
        rotate(2,1) = -1.0f;
        rotate(3,3) =  1.0f;
    }

    global_transform_ = Mat4f::scale(global_scale_) * rotate;

    return true;
}

bool DAE_loader::read_lights_lib(TiXmlElement *e)
{
    e = e->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "light")
            read_light(e);

        e = e->NextSiblingElement();
    }

    return true;
}

bool DAE_loader::read_light(TiXmlElement *e)
{
    TiXmlElement *tmp_e1,*tmp_e2,*extra_e,*tc_e,*lighttype_e;
    std::stringstream ss;
    std::string str;
    DAE_Light light;

    if (e->Attribute("id"))
    {
        light.id_ = e->Attribute("id");
    }

    e = e->FirstChildElement();

    tc_e = extra_e = lighttype_e = NULL;

    while (e)
    {
        if (e->ValueStr() == "technique_common")
            tc_e = e;
        else if (e->ValueStr() == "extra")
            extra_e = e;

        e = e->NextSiblingElement();
    }

    if (tc_e)
    {
        lighttype_e = tc_e->FirstChildElement();
        while (lighttype_e)
        {
            //find element with eiter directional, point or spot light data
            if (lighttype_e->ValueStr() == "directional")
                break;
            else if (lighttype_e->ValueStr() == "point")
                break;
            else if (lighttype_e->ValueStr() == "spot")
                break;

            lighttype_e = lighttype_e->NextSiblingElement();
        }
    }

    //lighttype_e is either directional, point, spot or NULL
    if (lighttype_e)
    {
        //read data from lighttype element
        tmp_e1 = lighttype_e->FirstChildElement();
        while (tmp_e1)
        {
            if (tmp_e1->ValueStr() == "color")
            {
                ss.clear();
                ss.str(tmp_e1->GetText());
                ss >> light.color_[0] >> light.color_[1] >> light.color_[2];
            }
            else if (tmp_e1->ValueStr() == "constant_attenuation")
            {
                ss.clear();
                ss.str(tmp_e1->GetText());
                ss >> light.attenuation_[0];
            }
            else if (tmp_e1->ValueStr() == "linear_attenuation")
            {
                ss.clear();
                ss.str(tmp_e1->GetText());
                ss >> light.attenuation_[1];
            }
            else if (tmp_e1->ValueStr() == "quadratic_attenuation")
            {
                ss.clear();
                ss.str(tmp_e1->GetText());
                ss >> light.attenuation_[2];
            }

            tmp_e1 = tmp_e1->NextSiblingElement();
        }

        //for (old) maya export (octavis supermarket for example) read FCOLLADA profile
        //may add additional profiles later?!
        if (extra_e)
        {
            tmp_e1 = extra_e->FirstChildElement();
            while (tmp_e1)
            {
                if (tmp_e1->ValueStr() == "technique")
                {
                    str = tmp_e1->Attribute("profile");

                    if (str == "FCOLLADA") // FBX COLLADA profile
                    {
                        tmp_e2 = tmp_e1->FirstChildElement();
                        while (tmp_e2)
                        {
                            if (tmp_e2->ValueStr() == "intensity")
                            {
                                ss.clear();
                                ss.str(tmp_e2->GetText());
                                float intensity;
                                ss >> intensity;
                                light.color_ *= intensity;
                            }

                            tmp_e2 = tmp_e2->NextSiblingElement();
                        }
                    }

                }
                tmp_e1 = tmp_e1->NextSiblingElement();
            }

        }

        light_lib_[light.id_] = light;
    }


    return true;
}

bool DAE_loader::read_images_lib(TiXmlElement *e)
{
    e = e->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "image")
            read_image(e);
        e = e->NextSiblingElement();
    }
    return true;
}

bool DAE_loader::read_image(TiXmlElement *e)
{
    TiXmlElement* tmp_e;
    std::string full_filename, rel_filename, texture_name;

    texture_name = e->Attribute("id");

    if (texture_name.empty())
        return false;

    e = e->FirstChildElement();
    while (e)
    {
        if (e->ValueStr() == "init_from")
        {
            tmp_e = e->FirstChildElement();
            if (tmp_e)
            {
                if (tmp_e->ValueStr() == "ref")
                {
                    rel_filename = tmp_e->GetText();
                }
            }
            else
            {
                rel_filename = e->GetText();
            }
            break;
        }
        e = e->NextSiblingElement();
    }

    size_t pos = filename_.rfind('/');
    if (pos == std::string::npos)
    {
        pos = filename_.rfind('\\');
    }

    filename_.substr(0, pos + 1);

    full_filename = filename_.substr(0, pos + 1) + rel_filename;

    texture_lib_[texture_name] = full_filename;

    return true;
}

bool DAE_loader::read_materials_lib(TiXmlElement *e)
{
    std::string str;
    std::string id;
    TiXmlElement* tmp_e;

    e = e->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "material")
        {
            id.clear();
            id = e->Attribute("id");

            if (! id.empty())
            {
                tmp_e = e->FirstChildElement();
                while (tmp_e)
                {
                    if (tmp_e->ValueStr() == "instance_effect")
                    {
                        str = tmp_e->Attribute("url");
                        str = str.substr(1);
                        if (effect_lib_.count(str) > 0)
                        {
                            materialid_to_effect_[id] = effect_lib_[str];
                        }
                        else
                        {
                            std::cout << "DAE_loader::read_materials_lib: INFO Could not find instance_effect \""
                                         << str << "\"" << std::endl;
                        }
                        break;
                    }
                    tmp_e = tmp_e->NextSiblingElement();
                }
            }
        }

        e = e->NextSiblingElement();
    }
    return true;
}

bool DAE_loader::read_effects_lib(TiXmlElement *e)
{
    DAE_Effect* effect;

    e = e->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "effect")
        {
            effect = new DAE_Effect;
            effect->id_ = e->Attribute("id");
            if (read_effect(e, effect))
            {
                effect_lib_[effect->id_] = effect;
            }
            else
            {
                delete effect;
            }
        }

        e = e->NextSiblingElement();
    }
    return true;
}

bool DAE_loader::read_effect(TiXmlElement *e, DAE_Effect *effect)
{
    TiXmlElement *tmp_e1, *tmp_e2, *tmp_e3, *tmp_e4;
    std::stringstream ss;
    bool found_profile_common = false;
    std::string texture_name;


    e = e->FirstChildElement();

    //find profile_*
    while (e)
    {
        if (e->ValueStr() == "profile_COMMON")
        {
            found_profile_common = true;

            tmp_e1 = get_sibling_by_name(e->FirstChildElement(), "technique");

            if (tmp_e1)
            {
                tmp_e1 = tmp_e1->FirstChildElement();

                while (tmp_e1)
                {
                    if (tmp_e1->ValueStr() == "phong"   ||
                        tmp_e1->ValueStr() == "lambert" ||
                        tmp_e1->ValueStr() == "blinn"   ||
                        tmp_e1->ValueStr() == "constant")
                    {
                        tmp_e2 = tmp_e1->FirstChildElement();
                        while (tmp_e2)
                        {
                            if (tmp_e2->ValueStr() == "emission")
                            {
                                tmp_e3 = tmp_e2->FirstChildElement();

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "color");
                                if(tmp_e3)
                                {
                                    ss.clear(); ss.str(tmp_e3->GetText());
                                    ss      >> effect->emission_color_[0]
                                            >> effect->emission_color_[1]
                                            >> effect->emission_color_[2];
                                }
                            }
                            else if (tmp_e2->ValueStr() == "ambient")
                            {
                                tmp_e3 = tmp_e2->FirstChildElement();

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "color");
                                if(tmp_e3)
                                {
                                    ss.clear(); ss.str(tmp_e3->GetText());
                                    ss      >> effect->ambient_color_[0]
                                            >> effect->ambient_color_[1]
                                            >> effect->ambient_color_[2];
                                }
                            }
                            else if (tmp_e2->ValueStr() == "diffuse")
                            {
                                tmp_e3 = tmp_e2->FirstChildElement();

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "color");
                                if(tmp_e3)
                                {
                                    ss.clear(); ss.str(tmp_e3->GetText());
                                    ss      >> effect->diffuse_color_[0]
                                            >> effect->diffuse_color_[1]
                                            >> effect->diffuse_color_[2];
                                }

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "texture");
                                if (tmp_e3)
                                {
                                    read_texture(tmp_e3, e, effect, texture_name);
                                    effect->texture_filename_ = texture_name;
                                }
                            }
                            else if (tmp_e2->ValueStr() == "specular")
                            {
                                tmp_e3 = tmp_e2->FirstChildElement();

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "color");
                                if(tmp_e3)
                                {
                                    ss.clear(); ss.str(tmp_e3->GetText());
                                    ss      >> effect->specular_color_[0]
                                            >> effect->specular_color_[1]
                                            >> effect->specular_color_[2];
                                }

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "texture");
                                if (tmp_e3)
                                {
                                    read_texture(tmp_e3, e, effect, texture_name);
                                    effect->texture_filename_specmap_ = texture_name;
                                }
                            }
                            else if (tmp_e2->ValueStr() == "shininess")
                            {
                                tmp_e3 = tmp_e2->FirstChildElement();

                                tmp_e3 = get_sibling_by_name(tmp_e2->FirstChildElement(), "float");
                                if(tmp_e3)
                                {
                                    ss.clear(); ss.str(tmp_e3->GetText());
                                    ss      >> effect->specular_color_[3];
                                }
                            }

                            tmp_e2 = tmp_e2->NextSiblingElement();
                        }
                    }
                    else if (tmp_e1->ValueStr() == "extra")
                    {
                        tmp_e2 = tmp_e1->FirstChildElement();
                        while (tmp_e2)
                        {
                            tmp_e3 = tmp_e2->FirstChildElement();
                            while (tmp_e3)
                            {
                                if (tmp_e3->ValueStr() == "bump")
                                {
                                    tmp_e4 = get_sibling_by_name(tmp_e3->FirstChildElement(), "texture");
                                    if (tmp_e4)
                                    {
                                        read_texture(tmp_e4, e, effect, texture_name);
                                        effect->texture_filename_normalmap_ = texture_name;
                                    }
                                }

                                tmp_e3 = tmp_e3->NextSiblingElement();
                            }
                            tmp_e2 = tmp_e2->NextSiblingElement();
                        }
                    }
                    else
                    {
                        return false;
                    }

                    tmp_e1 = tmp_e1->NextSiblingElement();
                }
            }
        }

        e = e->NextSiblingElement();
    }


    return found_profile_common;
}


bool DAE_loader::read_texture(TiXmlElement *texture_e, TiXmlElement* base_e, DAE_Effect *effect, std::string& texture_name)
{
    TiXmlElement* tmp_e;
    std::stringstream ss;

    //find *-sampler
    tmp_e = get_sibling_by_attr_value(base_e->FirstChildElement(), "sid", texture_e->Attribute("texture"));
    if (tmp_e)
    {
        tmp_e = tmp_e->FirstChildElement();

        //find source of sampler
        tmp_e = get_sibling_by_name(tmp_e->FirstChildElement(), "source");
        if (tmp_e)
        {
            //source is "Text" of source-element
            tmp_e = get_sibling_by_attr_value(base_e->FirstChildElement(), "sid", tmp_e->GetText());
            if (tmp_e)
            {
                //find *-surface
                tmp_e = get_sibling_by_name(tmp_e->FirstChildElement(), "surface");
                if (tmp_e)
                {
                    //find init_from element
                    tmp_e = get_sibling_by_name(tmp_e->FirstChildElement(), "init_from");
                    if (tmp_e)
                    {
                        if (texture_lib_.count(tmp_e->GetText()) > 0)
                        {
                            //init_from contains image name as "Text"
                            texture_name = texture_lib_[tmp_e->GetText()];

                            //look for "extra" to read texture repeat and other information
                            tmp_e = get_sibling_by_name(texture_e->FirstChildElement(), "extra");
                            if (tmp_e)
                            {
                                tmp_e = tmp_e->FirstChildElement()->FirstChildElement();
                                while(tmp_e)
                                {
                                    if (tmp_e->ValueStr() == "repeatU")
                                    {
                                        ss.clear(); ss.str(tmp_e->GetText());
                                        ss >> effect->texture_repeat_[0];
                                    }
                                    else if (tmp_e->ValueStr() == "repeatV")
                                    {
                                        ss.clear(); ss.str(tmp_e->GetText());
                                        ss >> effect->texture_repeat_[1];
                                    }

                                    tmp_e = tmp_e->NextSiblingElement();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}


bool DAE_loader::read_geometries_lib(TiXmlElement *e)
{
    e = e->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "geometry")
            read_geometry(e);

        e = e->NextSiblingElement();
    }

    return true;
}

bool DAE_loader::read_geometry(TiXmlElement *e)
{
    TiXmlElement *e1;
    DAE_Mesh* mesh=NULL;

    e1 = e->FirstChildElement();

    while (e1)
    {
        if (e1->ValueStr() == "mesh")
        {
            mesh = new DAE_Mesh;
            mesh->id_ = e->Attribute("id");
            if (read_mesh(e1, mesh))
            {
                mesh_lib_[mesh->id_] = mesh;
            }
            else
            {
                delete mesh;
                mesh = NULL;
            }
        }
        e1 = e1->NextSiblingElement();
    }


    return mesh != NULL;
}

bool DAE_loader::read_mesh(TiXmlElement *e, DAE_Mesh *mesh)
{
    TiXmlElement* tmp_e1,*tmp_e2, *triangles_e=NULL, *vertices_e=NULL;
    std::vector<TiXmlElement*> triangles_elements;
    std::string str;
    std::stringstream ss;
    size_t i;
    int offset;
    int count;
    unsigned int idx;


    tmp_e1 = e->FirstChildElement();

    while (tmp_e1)
    {
        if (tmp_e1->ValueStr() == "vertices")
        {
            vertices_e = tmp_e1;
        }
        else if (tmp_e1->ValueStr() == "triangles")
        {
            triangles_elements.push_back(tmp_e1);
        }
        else if (tmp_e1->ValueStr() == "polylist")
        {
            //TODO: do this correct or at least check "vcount" element if it is always 3
            triangles_elements.push_back(tmp_e1);
        }

        tmp_e1 = tmp_e1->NextSiblingElement();
    }

    if (vertices_e == NULL)
        return false;

    tmp_e1 = vertices_e->FirstChildElement();
    while (tmp_e1)
    {
        if (tmp_e1->ValueStr() == "input")
        {
            str = tmp_e1->Attribute("semantic");
            if (str == "POSITION")
            {
                str = tmp_e1->Attribute("source");
                str = str.substr(1);//remove #
                tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                if (tmp_e2)
                {
                    read_source(tmp_e2, &count, &mesh->vertex_stride_, mesh->vertices_);
                }
            }
            else if (str == "NORMAL")
            {
                str = tmp_e1->Attribute("source");
                str = str.substr(1);//remove #
                tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                if (tmp_e2)
                {
                    read_source(tmp_e2, &count, &mesh->normal_stride_, mesh->normals_);
                }
            }
            else if (str == "TEXCOORD")
            {
                if (mesh->texcoords_.empty())
                {
                    str = tmp_e1->Attribute("source");
                    str = str.substr(1);//remove #
                    tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                    if (tmp_e2)
                    {
                        read_source(tmp_e2, &count, &mesh->texcoord_stride_, mesh->texcoords_);
                    }
                }
            }
        }

        tmp_e1 = tmp_e1->NextSiblingElement();
    }

    mesh->index_step_ = 0;


    for (i=0; i < triangles_elements.size(); ++i)
    {
        triangles_e = triangles_elements[i];

        tmp_e1 = triangles_e->FirstChildElement();
        while (tmp_e1)
        {
            if (tmp_e1->ValueStr() == "input")
            {
                str = tmp_e1->Attribute("semantic");
                offset = -1;
                tmp_e1->Attribute("offset", &offset);
                mesh->index_step_ = (mesh->index_step_ > offset) ? mesh->index_step_ : offset;
                if (str == "VERTEX")
                {
                    //source must be "vertices" element
                    //just get index offset
                    mesh->vertex_offset_ = offset;
                }
                else if (str == "NORMAL")
                {
                    mesh->normal_offset_ = offset;
                    str = tmp_e1->Attribute("source");
                    str = str.substr(1);
                    tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                    if (tmp_e2)
                    {
                        read_source(tmp_e2, &count, &mesh->normal_stride_, mesh->normals_);
                    }
                }
                else if (str == "TEXCOORD")
                {
                    if (mesh->texcoords_.empty())
                    {
                        mesh->texcoord_offset_ = offset;
                        str = tmp_e1->Attribute("source");
                        str = str.substr(1);
                        tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                        if (tmp_e2)
                        {
                            read_source(tmp_e2, &count, &mesh->texcoord_stride_, mesh->texcoords_);
                        }
                    }
                }
                else if (str == "COLOR")
                {
                    mesh->color_offset_ = offset;
                    str = tmp_e1->Attribute("source");
                    str = str.substr(1);
                    tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                    if (tmp_e2)
                    {
                        read_source(tmp_e2, &count, &mesh->color_stride_, mesh->colors_);
                    }
                }
            }
            else if (tmp_e1->ValueStr() == "p")
            {
                if (tmp_e1->GetText())
                {
                    ss.clear();
                    ss.str(tmp_e1->GetText());

                    while (ss.good())
                    {
                        ss >> idx;
                        mesh->indices_.push_back(idx);
                    }
                }
            }

            tmp_e1 = tmp_e1->NextSiblingElement();
        }
    }

    return true;
}


bool DAE_loader::read_controllers_lib(TiXmlElement *e)
{
    e = e->FirstChildElement();

    while (e)
    {
        if (e->ValueStr() == "controller")
            read_controller(e);

        e = e->NextSiblingElement();
    }

    return true;
}

bool DAE_loader::read_controller(TiXmlElement *e)
{
    TiXmlElement* e1 = e->FirstChildElement();

    while (e1)
    {
        if (e1->ValueStr() == "morph")
        {
            DAE_Morph* morph;
            if (e->Attribute("id"))
            {
                morph = new DAE_Morph;
                morph->id_ = e->Attribute("id");
                if (read_morph(e1,morph))
                {
                    morph_lib_[morph->id_] = morph;
                }
                else
                {
                    delete morph;
                }
            }
            else
            {
                std::cout << "DAE_loader::read_controller: [WARNING] Missing id for morph. Skipping!" << std::endl;
            }
        }
        else if (e1->ValueStr() == "skin")
        {
            DAE_Skin* skin;
            if (e->Attribute("id"))
            {
                skin = new DAE_Skin;
                skin->id_ = e->Attribute("id");
                if (read_skin(e1, skin))
                {
                    skin_lib_[skin->id_] = skin;
                }
                else
                {
                    delete skin;
                }
            }
            else
            {
                std::cout << "DAE_loader::read_controller: [WARNING] Missing id for skin. Skipping!" << std::endl;
            }
        }

        e1 = e1->NextSiblingElement();
    }

    return true;
}

bool DAE_loader::read_morph(TiXmlElement *e, DAE_Morph *morph)
{
    TiXmlElement* tmp_e1,*tmp_e2;
    std::string str;
    int count, stride;
    std::vector<std::string> string_array;
    std::vector<float> float_array;
    size_t i;

    //get id of base mesh
    if (e->Attribute("source")) // collada 1.5
        str = e->Attribute("source");
    else if (e->Attribute("sid")) // collada 1.4.1
        str = e->Attribute("sid");
    else
        return false;

    if (str[0] == '#')
        str = str.substr(1);

    morph->base_mesh_id_ = str;


    //get targets
    tmp_e1 = get_sibling_by_name(e->FirstChildElement(), "targets");

    if (tmp_e1)
    {
        tmp_e1 = tmp_e1->FirstChildElement();
        while (tmp_e1)
        {
            if (tmp_e1->ValueTStr() == "input" && tmp_e1->Attribute("semantic"))
            {
                str = tmp_e1->Attribute("semantic");
                if (str == "MORPH_TARGET")
                {
                    str = tmp_e1->Attribute("source");
                    tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str.substr(1));

                    string_array.clear();
                    read_source(tmp_e2, &count, &stride, string_array);
                    for (i=0; i < string_array.size(); ++i)
                    {
                        morph->target_ids_.push_back(string_array[i]);
                    }
                }
                else if (str == "MORPH_WEIGHT")
                {
                    str = tmp_e1->Attribute("source");
                    tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str.substr(1));

                    float_array.clear();
                    read_source(tmp_e2, &count, &stride, float_array);

                    morph->morph_weights_ = float_array;
                }
            }

            tmp_e1 = tmp_e1->NextSiblingElement();
        }
    }
    else
    {
        return false;
    }



    return true;
}

bool DAE_loader::read_skin(TiXmlElement *e, DAE_Skin* skin)
{

    std::string str;
    int count, stride,val,offset;
    TiXmlElement* tmp_e1,*tmp_e2;
    std::stringstream ss;


    //get id of base mesh
    if (e->Attribute("source")) // collada 1.5
        str = e->Attribute("source");
    else if (e->Attribute("sid")) // collada 1.4.1
        str = e->Attribute("sid");
    else
        return false;

    str = str.substr(1);

    skin->target_id_ = str;

    tmp_e1 = e->FirstChildElement();
    tmp_e1 = get_sibling_by_name(e->FirstChildElement(), "vertex_weights");

    tmp_e1 = tmp_e1->FirstChildElement();

    skin->index_step_ = 1;

    while (tmp_e1)
    {
        if (tmp_e1->ValueStr() == "input")
        {
            offset = 0;
            tmp_e1->Attribute("offset", &offset);
            skin->index_step_ = (skin->index_step_ > offset) ? skin->index_step_ : offset;

            str = tmp_e1->Attribute("semantic");
            if (str == "JOINT")
            {
                str = tmp_e1->Attribute("source");
                str = str.substr(1); // remove #
                skin->joints_offset_ = offset;
                tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                if (tmp_e2)
                {
                    read_source(tmp_e2, &count, &stride, skin->joints_array_);
                }
            }
            else if (str == "WEIGHT")
            {
                str = tmp_e1->Attribute("source");
                str = str.substr(1);
                skin->weights_offset_ = offset;
                tmp_e2 = get_sibling_by_attr_value(e->FirstChildElement(), "id", str);
                if (tmp_e2)
                {
                    read_source(tmp_e2, &count, &stride, skin->weights_array_);
                }
            }

        }
        else if (tmp_e1->ValueStr() == "vcount")
        {
            ss.clear();
            ss.str(tmp_e1->GetText());

            while (ss.good())
            {
                ss >> val;
                skin->vcount_.push_back(val);
            }
        }
        else if (tmp_e1->ValueStr() == "v")
        {
            ss.clear();
            ss.str(tmp_e1->GetText());

            while (ss.good())
            {
                ss >> val;
                skin->v_.push_back(val);
            }
        }

        tmp_e1 = tmp_e1->NextSiblingElement();
    }


    return true;
}

bool DAE_loader::read_visual_scenes_lib(TiXmlElement *e)
{
    TiXmlElement* tmp_e;

    tmp_e = e->FirstChildElement();

    while (tmp_e)
    {
        read_visual_scene(tmp_e);
        tmp_e = tmp_e->NextSiblingElement();
    }

    return true;
}

bool DAE_loader::read_visual_scene(TiXmlElement *e)
{
    Mat4f transform;

    visual_scene_node_lib_.clear();
    build_node_lib_recursive(e);

    e = e->FirstChildElement();

    while (e)
    {
        transform = Mat4f::identity();
        read_visual_scene_node_recursive(e, transform);

        e = e->NextSiblingElement();
    }

    return true;
}

void DAE_loader::read_visual_scene_node_recursive(TiXmlElement *e, Mat4f& transform)
{
    std::string str;
    TiXmlElement* tmp_e;
    Mat4f current_transform;
    DAE_Skeleton* skeleton;


    if (e->Attribute("type"))
        str = e->Attribute("type");

    if (str == "JOINT") //if JOINT: read skeleton
    {
        if (e->Attribute("id"))
        {
            str = e->Attribute("id");
            skeleton = new DAE_Skeleton;
            read_joint_node_recursive(e,NULL, skeleton);
            skeletons_.push_back(skeleton);
        }
    }
    else //else: read other stuff
    {
        tmp_e = e->FirstChildElement();

        current_transform = transform * get_local_transform(tmp_e);

        while (tmp_e)
        {
            if (tmp_e->ValueStr() == "node")
            {
                read_visual_scene_node_recursive(tmp_e, current_transform);
            }
            else if (tmp_e->ValueStr() == "instance_node")
            {
                str = tmp_e->Attribute("url");
                str = str.substr(1);
                if (visual_scene_node_lib_.count(str) > 0)
                {
                    read_visual_scene_node_recursive(visual_scene_node_lib_[str], current_transform);
                }
                else
                {
                    std::cout << "DAE_loader::read_visual_scene_node_recursive: INFO Could not find instance_node \""
                                 << str << "\"" << std::endl;
                }
            }
            else if (tmp_e->ValueStr() == "instance_light")
            {
                read_instance_light(tmp_e, current_transform);
            }
            else if (tmp_e->ValueStr() == "instance_geometry")
            {
                read_instance_geometry(tmp_e, current_transform );
            }
            else if (tmp_e->ValueStr() == "instance_controller")
            {
                read_instance_controller(tmp_e, current_transform);
            }

            tmp_e = tmp_e->NextSiblingElement();
        }
    }

}

void DAE_loader::read_instance_geometry(TiXmlElement *e, const Mat4f &transform)
{
    DAE_Mesh* mesh;
    DAE_Effect* effect;
    std::string str;

    str = e->Attribute("url");
    str = str.substr(1);
    if (mesh_lib_.count(str) > 0)
    {
        mesh = mesh_lib_[str];
        mesh->instances_.push_back(global_transform_ * transform);

        e = get_element_by_name_recursive(e->FirstChildElement(), "instance_material");
        if (e)
        {
            str = e->Attribute("target");
            str = str.substr(1);

            //TODO: do this correctly?!
            // read instance effects and divide same meshes with different effects into separate meshes
            if (materialid_to_effect_.count(str) > 0)
            {
                effect = materialid_to_effect_[str];
                mesh->effect_ = effect;
            }
            else
            {
                std::cout << "DAE_loader::read_instance_geometry: INFO Could not find instance_material \""
                          << str << "\"" << std::endl;
            }
        }
    }
    else
    {
        std::cout << "DAE_loader::read_instance_geometry: INFO Could not find instance_geometry \""
                     << str << "\"" << std::endl;
    }
}

void DAE_loader::read_instance_light(TiXmlElement *e, const Mat4f &transform)
{
    std::string url;

    url = e->Attribute("url");
    //remove '#'
    url = url.substr(1);

    DAE_Light light;

    if (light_lib_.count(url) > 0)
    {
        light = light_lib_[url];
        light.positions_.push_back(affine_transform(global_transform_ * transform,  Vec3f(0.0f,0.0f,0.0f) ));
    }

}

void DAE_loader::read_instance_controller(TiXmlElement *e, const Mat4f &transform)
{
    std::string str;

    str = e->Attribute("url");
    //remove '#'
    str = str.substr(1);


    if (morph_lib_.count(str) > 0)
    {
        DAE_Morph* morph;
        morph = morph_lib_[str];

        morph->instances_.push_back(global_transform_ * transform);
        e = get_element_by_name_recursive(e->FirstChildElement(), "instance_material");
        if (e)
        {
            str = e->Attribute("target");
            str = str.substr(1);

            if (materialid_to_effect_.count(str) > 0)
            {
                morph->effect_ = materialid_to_effect_[str];
            }
            else
            {
                std::cout << "DAE_loader::read_instance_controller: INFO Could not find instance_material \""
                          << str << "\"" << std::endl;
            }
        }
    }
    else if (skin_lib_.count(str) > 0)
    {
        DAE_Skin* skin;
        skin = skin_lib_[str];

        skin->instances_.push_back(global_transform_ * transform);

        e = get_element_by_name_recursive(e->FirstChildElement(), "instance_material");
        if (e)
        {
            str = e->Attribute("target");
            str = str.substr(1);

            if (materialid_to_effect_.count(str) > 0)
            {
                skin->effect_ = materialid_to_effect_[str];
            }
            else
            {
                std::cout << "DAE_loader::read_instance_controller: INFO Could not find instance_material \""
                          << str << "\"" << std::endl;
            }
        }
    }
}

void DAE_loader::read_joint_node_recursive(TiXmlElement *e, DAE_Joint *parent, DAE_Skeleton *skeleton)
{
    Mat4f local_transform = Mat4f::identity();
    TiXmlElement* tmp_e1;
    DAE_Joint* new_joint;
    std::string str;

    tmp_e1 = e->FirstChildElement();

    if (tmp_e1)
    {
        local_transform = get_local_transform(tmp_e1);

        if (e->Attribute("id"))
        {
            str = e->Attribute("id");

            //apply global scale
            local_transform(0,3) *= global_scale_;
            local_transform(1,3) *= global_scale_;
            local_transform(2,3) *= global_scale_;

            new_joint = new DAE_Joint;
            new_joint->transform_ = local_transform;
            new_joint->name_ = str;

            new_joint->parent_ = parent;

            if (parent)
            {
                parent->children_.push_back(new_joint);
            }
            else
            {
                skeleton->root_ = new_joint;
            }

            skeleton->joints_.push_back(new_joint);
        }

        while (tmp_e1)
        {
            if (tmp_e1->ValueStr() == "node")
                read_joint_node_recursive(tmp_e1, new_joint, skeleton);

            tmp_e1 = tmp_e1->NextSiblingElement();
        }
    }

}

bool DAE_loader::read_source(TiXmlElement *e, int *count, int *stride, std::vector<float> &float_array)
{
    TiXmlElement *e1,*e2;
    bool has_accessor = false;
    size_t i;
    int num_values=0;
    std::stringstream ss;

    *stride = 1;
    *count = 0;
    float_array.clear();

    e1 = e->FirstChildElement();

    while (e1)
    {
        e2 = e1->FirstChildElement();
        if (e2 && e2->ValueStr() == "accessor")
        {
            has_accessor = true;

            e2->Attribute("count", count);
            e2->Attribute("stride", stride);

            break;
        }

        e1 = e1->NextSiblingElement();
    }


    e1 = get_sibling_by_name(e->FirstChildElement(), "float_array");
    if (e1 && e1->GetText() != NULL)
    {
        ss.str(e1->GetText());

        e1->Attribute("count", &num_values);
        float_array.resize(num_values);

        for (i=0; i < float_array.size(); ++i)
        {
            ss >> float_array[i];
        }
    }
/*
    e1 = get_sibling_by_name(e->FirstChildElement(), "int_array");
    if (e1)
    {
        ss.str(e1->GetText());

        e1->Attribute("count", &num_values);
        int_array.resize(num_values);

        for (i=0; i < int_array.size(); ++i)
        {
            ss >> int_array[i];
        }
    }
*/
    if (!has_accessor)
    {
        *count = num_values;
        *stride = 1;
    }

    return true;
}

bool DAE_loader::read_source(TiXmlElement *e, int *count, int *stride, std::vector<std::string> &string_array)
{
    TiXmlElement *e1,*e2;
    bool has_accessor = false;
    size_t i;
    int num_values=0;
    std::stringstream ss;

    *stride = 1;
    *count = 0;
    string_array.clear();

    e1 = e->FirstChildElement();

    while (e1)
    {
        e2 = e1->FirstChildElement();
        if (e2 && e2->ValueStr() == "accessor")
        {
            has_accessor = true;

            e2->Attribute("count", count);
            e2->Attribute("stride", stride);

            break;
        }

        e1 = e1->NextSiblingElement();
    }


    e1 = get_sibling_by_name(e->FirstChildElement(), "IDREF_array");
    if (e1)
    {
        ss.str(e1->GetText());

        e1->Attribute("count", &num_values);
        string_array.resize(num_values);

        for (i=0; i < string_array.size(); ++i)
        {
            ss >> string_array[i];
        }
    }
    else
    {
        e1 = get_sibling_by_name(e->FirstChildElement(), "Name_array");
        if (e1)
        {
            ss.str(e1->GetText());

            e1->Attribute("count", &num_values);
            string_array.resize(num_values);

            for (i=0; i < string_array.size(); ++i)
            {
                ss >> string_array[i];
            }
        }
    }


    if (!has_accessor)
    {
        *count = num_values;
        *stride = 1;
    }

    return true;
}


void DAE_loader::build_node_lib_recursive(TiXmlElement *e)
{
    while (e)
    {
        if (e->ValueStr() == "node")
        {
            visual_scene_node_lib_[e->Attribute("id")] = e;
        }
        build_node_lib_recursive(e->FirstChildElement());

        e = e->NextSiblingElement();
    }
}



const Mat4f DAE_loader::get_local_transform(TiXmlElement *e)
{
    std::stringstream ss;

    Vec3f v3;
    Vec4f v4;
    Mat4f result = Mat4f::identity();

    while (e)
    {
        if (e->ValueStr() == "matrix")
        {
            int i,j;
            Mat4f tmp_mat;
            ss.clear(); ss.str(e->GetText());
            for (i=0; i < 4; ++i)
            {
                for (j=0; j < 4; ++j)
                {
                    ss >> tmp_mat(i,j);
                }
            }
            result = result*tmp_mat;
        }
        else if (e->ValueStr() == "translate")
        {
            ss.clear(); ss.str(e->GetText());

            ss      >> v3[0]
                    >> v3[1]
                    >> v3[2];
            result = result * Mat4f::translate(v3);
        }
        else if (e->ValueStr() == "rotate")
        {
            ss.clear(); ss.str(e->GetText());
            ss      >> v4[0]
                    >> v4[1]
                    >> v4[2]
                    >> v4[3];
            result = result * Mat4f::rotate(v4);

        }
        else if (e->ValueStr() == "scale")
        {
            ss.clear(); ss.str(e->GetText());
            ss      >> v3[0]
                    >> v3[1]
                    >> v3[2];
            result = result * Mat4f::scale(v3);
        }

        e = e->NextSiblingElement();
    }
    return result;
}

TiXmlElement* DAE_loader::get_sibling_by_name(TiXmlElement *e, const std::string& name)
{
    while (e != NULL)
    {
        if (e->ValueStr() == name)
            return e;

        e = e->NextSiblingElement();
    }
    return NULL;
}


TiXmlElement* DAE_loader::get_sibling_by_attr_value(TiXmlElement *e, const std::string &attr, const std::string &val)
{
    const std::string *a;
    while (e != NULL)
    {
        a = e->Attribute(attr);
        if (a != NULL && *a == val)
            return e;

        e = e->NextSiblingElement();
    }
    return NULL;
}

TiXmlElement* DAE_loader::get_element_by_name_recursive(TiXmlElement *e, const std::string &name)
{
    TiXmlElement* result = NULL;
    while (e)
    {
        if (e->ValueStr() == name)
        {
            return e;
        }

        result = get_element_by_name_recursive(e->FirstChildElement(), name);

        e = e->NextSiblingElement();
    }

    return result;
}


} //namespace character
} //namespace graphene
