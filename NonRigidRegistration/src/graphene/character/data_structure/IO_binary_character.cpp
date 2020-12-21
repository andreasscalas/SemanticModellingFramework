//=============================================================================

#include "IO.h"
#include "BIM_file.h"
#include <fstream>
#include <graphene/character/data_structure/Character.h>
#include <typeinfo>


//=============================================================================

using namespace graphene::surface_mesh;

namespace graphene {
namespace character {

//=============================================================================


template <class T>
void memcpy_vecT(std::vector<T>& dst, const std::string& src)
{
    dst.clear();
    dst.resize(src.size()/sizeof(T));
    memcpy(&dst[0], src.c_str(), src.size());
}


bool bim_mesh_to_mesh(const bim::BIM_file::BIM_mesh &bim_mesh, Surface_mesh* mesh, const std::string& filename)
{
    //temporary data vectors
    std::vector<unsigned int> indices;
    std::vector<Point> points;
    std::vector<Normal> h_normals;
    std::vector<Normal> v_normals;
    std::vector<Color>  v_colors;
    std::vector<Texture_coordinate> h_texcoords;
    std::vector<Vec4f> v_weights;
    std::vector<Vec4f> v_depends;
    std::vector<Vec4f> v_weights2;
    std::vector<Vec4f> v_depends2;
    std::vector<Vec3f> v_cor;

    //mesh properties
    Surface_mesh::Vertex_property<Point>  sm_vertices_;
    Surface_mesh::Vertex_property<Normal> sm_v_normals_;
    Surface_mesh::Vertex_property<Color>  sm_v_colors_;
    Surface_mesh::Vertex_property<Vec4f>  sm_v_weights_;
    Surface_mesh::Vertex_property<Vec4f>  sm_v_depends_;
    Surface_mesh::Vertex_property<Vec4f>  sm_v_weights2_;
    Surface_mesh::Vertex_property<Vec4f>  sm_v_depends2_;
    Surface_mesh::Vertex_property<Vec3f>  sm_v_cor_;

    Surface_mesh::Halfedge_property<Normal>              sm_h_normals_;
    Surface_mesh::Halfedge_property<Texture_coordinate>  sm_h_texcoords_;

    Surface_mesh::Mesh_property<std::string>  sm_texture;
    Surface_mesh::Mesh_property<std::string>  sm_texture_nm;
    Surface_mesh::Mesh_property<std::string>  sm_texture_specular;

    if (bim_mesh.semantic_to_data_.count("string diffuse_texture") > 0)
    {
        //extract dir
        std::string fname = filename;
        size_t pos = fname.rfind('/');
        if (pos == std::string::npos)
            pos = fname.rfind('\\');
        fname = fname.substr(0, pos + 1);

        fname += bim_mesh.semantic_to_data_.find("string diffuse_texture")->second;

        sm_texture = mesh->mesh_property<std::string>("m:texturename");
        sm_texture[0] = fname;
    }

    if (bim_mesh.semantic_to_data_.count("string normalmap_texture") > 0)
    {
        //extract dir
        std::string fname = filename;
        size_t pos = fname.rfind('/');
        if (pos == std::string::npos)
            pos = fname.rfind('\\');
        fname = fname.substr(0, pos + 1);

        fname += bim_mesh.semantic_to_data_.find("string normalmap_texture")->second;

        sm_texture_nm = mesh->mesh_property<std::string>("m:texturename_nm");
        sm_texture_nm[0] = fname;
    }

    if (bim_mesh.semantic_to_data_.count("string specularmap_texture") > 0)
    {
        //extract dir
        std::string fname = filename;
        size_t pos = fname.rfind('/');
        if (pos == std::string::npos)
            pos = fname.rfind('\\');
        fname = fname.substr(0, pos + 1);

        fname += bim_mesh.semantic_to_data_.find("string specularmap_texture")->second;

        sm_texture_specular = mesh->mesh_property<std::string>("m:texturename_spec");
        sm_texture_specular[0] = fname;
    }

    if (bim_mesh.semantic_to_data_.count("float3 ambient_color"))
    {
        Surface_mesh::Mesh_property<Vec3f> sm_ambient_color = mesh->mesh_property<Vec3f>("m:ambient_color");
        const std::string& data = bim_mesh.semantic_to_data_.find("float3 ambient_color")->second;
        memcpy(sm_ambient_color[0].data(), data.c_str(), sizeof(Vec3f));
    }

    if (bim_mesh.semantic_to_data_.count("float3 diffuse_color") > 0)
    {
        Surface_mesh::Mesh_property<Vec3f> sm_diffuse_color = mesh->mesh_property<Vec3f>("m:diffuse_color");
        const std::string& data = bim_mesh.semantic_to_data_.find("float3 diffuse_color")->second;
        memcpy(sm_diffuse_color[0].data(), data.c_str(), sizeof(Vec3f));
    }

    if (bim_mesh.semantic_to_data_.count("float4 specular_color") > 0)
    {
        Surface_mesh::Mesh_property<Vec4f> sm_specular_color = mesh->mesh_property<Vec4f>("m:specular_color");
        const std::string& data = bim_mesh.semantic_to_data_.find("float4 specular_color")->second;
        memcpy(sm_specular_color[0].data(), data.c_str(), sizeof(Vec4f));
    }

    if (bim_mesh.semantic_to_data_.count("float alpha") > 0)
    {
        Surface_mesh::Mesh_property<float> sm_alpha = mesh->mesh_property<float>("m:alpha");
        const std::string& data = bim_mesh.semantic_to_data_.find("float alpha")->second;
        memcpy(&sm_alpha[0], data.c_str(), sizeof(float));
    }

    if (bim_mesh.semantic_to_data_.count("float3vec vertices") > 0)
    {
        if (Point::size() == 3)
        {
            memcpy_vecT<Point>(points, bim_mesh.semantic_to_data_.find("float3vec vertices")->second);
            sm_vertices_ = mesh->get_vertex_property<Point>("v:point");
        }
    }

    if (! sm_vertices_ && bim_mesh.semantic_to_data_.count("float4vec vertices") > 0)
    {
        if (Point::size() == 4)
        {
            memcpy_vecT<Point>(points, bim_mesh.semantic_to_data_.find("float4vec vertices")->second);
            sm_vertices_ = mesh->get_vertex_property<Point>("v:point");
        }
    }

    //no vertices? no mesh!
    if (!sm_vertices_)
        return false;

    if (bim_mesh.semantic_to_data_.count("float3vec v_normals"))
    {
        memcpy_vecT<Normal>(v_normals, bim_mesh.semantic_to_data_.find("float3vec v_normals")->second);
        sm_v_normals_ = mesh->vertex_property<Normal>("v:normal");
    }

    if (bim_mesh.semantic_to_data_.count("float3vec v_colors"))
    {
        if (Color::size() == 3)
        {
            memcpy_vecT<Color>(v_colors, bim_mesh.semantic_to_data_.find("float3vec v_colors")->second);

            sm_v_colors_ = mesh->vertex_property<Color>("v:color");
        }
    }

    if (bim_mesh.semantic_to_data_.count("float4vec v_depends"))
    {
        memcpy_vecT<Vec4f>(v_depends, bim_mesh.semantic_to_data_.find("float4vec v_depends")->second);
        sm_v_depends_ = mesh->vertex_property<Vec4f>("v:skin_depend");
    }

    if (bim_mesh.semantic_to_data_.count("float4vec v_depends2"))
    {
        memcpy_vecT<Vec4f>(v_depends2, bim_mesh.semantic_to_data_.find("float4vec v_depends2")->second);
        sm_v_depends2_ = mesh->vertex_property<Vec4f>("v:skin_depend2");
    }

    if (bim_mesh.semantic_to_data_.count("float3vec v_cor"))
    {
        memcpy_vecT<Vec3f>(v_cor, bim_mesh.semantic_to_data_.find("float3vec v_cor")->second);
        sm_v_cor_ = mesh->vertex_property<Vec3f>("v:cor");
    }

    if (bim_mesh.semantic_to_data_.count("float4vec v_weights"))
    {
        memcpy_vecT<Vec4f>(v_weights, bim_mesh.semantic_to_data_.find("float4vec v_weights")->second);
        sm_v_weights_ = mesh->vertex_property<Vec4f>("v:skin_weight");
    }

    if (bim_mesh.semantic_to_data_.count("float4vec v_weights2"))
    {
        memcpy_vecT<Vec4f>(v_weights2, bim_mesh.semantic_to_data_.find("float4vec v_weights2")->second);
        sm_v_weights2_ = mesh->vertex_property<Vec4f>("v:skin_weight2");
    }

    /* TODO/ATTENTION: Comment this back in to load halfedge based normals*/
    if (bim_mesh.semantic_to_data_.count("float3vec fv_normals"))
    {
        memcpy_vecT<Normal>(h_normals, bim_mesh.semantic_to_data_.find("float3vec fv_normals")->second);
        sm_h_normals_ = mesh->halfedge_property<Normal>("h:normal");
    }


    if (bim_mesh.semantic_to_data_.count("float3vec fv_texcoords"))
    {
        if (Texture_coordinate::size() == 3)
        {
            memcpy_vecT<Texture_coordinate>(h_texcoords, bim_mesh.semantic_to_data_.find("float3vec fv_texcoords")->second);
            sm_h_texcoords_ = mesh->halfedge_property<Texture_coordinate>("h:texcoord");
        }
    }

    if (! sm_h_texcoords_ && bim_mesh.semantic_to_data_.count("float2vec fv_texcoords"))
    {
        if (Texture_coordinate::size() == 2)
        {
            memcpy_vecT<Texture_coordinate>(h_texcoords, bim_mesh.semantic_to_data_.find("float2vec fv_texcoords")->second);
            sm_h_texcoords_ = mesh->halfedge_property<Texture_coordinate>("h:texcoord");
        }
    }

    if (bim_mesh.semantic_to_data_.count("uintvec indices"))
    {
        memcpy_vecT<unsigned int>(indices, bim_mesh.semantic_to_data_.find("uintvec indices")->second);
    }


    //put all read data into corresponding Surface_mesh properties
    if (!indices.empty() && sm_vertices_)
    {
        Surface_mesh::Vertex v;
        Surface_mesh::Face f;
        Surface_mesh::Halfedge_around_face_circulator hfc, hfc_end;
        size_t i;
        int v_idx;

        mesh->reserve(points.size(), 3*points.size(), indices.size()/3);

        for (i=0; i < points.size(); ++i)
        {
            v = mesh->add_vertex(points[i]);

            if (sm_v_normals_ && v_normals.size() == points.size())
            {
                sm_v_normals_[v] = v_normals[i];
            }

            if (sm_v_colors_ && v_colors.size() == points.size())
            {
                sm_v_colors_[v] = v_colors[i];
            }

            if (sm_v_depends_ && v_depends.size() == points.size() )
            {
                sm_v_depends_[v] = v_depends[i];
            }

            if (sm_v_weights_ && v_weights.size() == points.size() )
            {
                sm_v_weights_[v] = v_weights[i];
            }

            if (sm_v_depends2_ && v_depends2.size() == points.size() )
            {
                sm_v_depends2_[v] = v_depends2[i];
            }

            if (sm_v_weights2_ && v_weights2.size() == points.size() )
            {
                sm_v_weights2_[v] = v_weights2[i];
            }

            if (sm_v_cor_ && v_cor.size() == points.size())
            {
                sm_v_cor_[v] = v_cor[i];
            }
        }

        //normalize weights so that they sum up to one
        if (sm_v_weights_ && sm_v_weights2_)
        {
            for (i=0; i < points.size(); ++i)
            {
                Vec4f &w1 = sm_v_weights_.vector()[i];
                Vec4f &w2 = sm_v_weights2_.vector()[i];

                const float r = 1.0f / (w1[0]+w1[1]+w1[2]+w1[3] + w2[0]+w2[1]+w2[2]+w2[3]);
                w1 *= r;
                w2 *= r;
            }
        }
        else if (sm_v_weights_)
        {
            for (i=0; i < points.size(); ++i)
            {
                Vec4f &w1 = sm_v_weights_.vector()[i];
                const float r = 1.0f / (w1[0]+w1[1]+w1[2]+w1[3]);
                w1 *= r;
            }
        }


        for (i=0; i < indices.size(); i+=3)
        {
            f = mesh->add_triangle(
                            Surface_mesh::Vertex((int) indices[i+0]),
                            Surface_mesh::Vertex((int) indices[i+1]),
                            Surface_mesh::Vertex((int) indices[i+2])
                        );

            hfc = hfc_end = mesh->halfedges(f);
            v_idx = 0;
            do
            {
                if (sm_h_normals_ && h_normals.size() == indices.size())
                {
                    sm_h_normals_[*hfc] = h_normals[i+v_idx];
                }

                if (sm_h_texcoords_ && h_texcoords.size() == indices.size())
                {
                    sm_h_texcoords_[*hfc] = h_texcoords[i+v_idx];
                }


                ++v_idx;
            }
            while (++hfc != hfc_end);

        }
    }
    else
    {
        return false;
    }

    if (!sm_h_normals_ && !sm_v_normals_)
    {
        mesh->update_face_normals();
        mesh->update_vertex_normals();
    }

    if (mesh->n_vertices() == 0 || mesh->n_faces() == 0)
    {
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------

bool bim_skeleton_to_skeleton(const bim::BIM_file::BIM_skeleton& bim_skeleton, Skeleton& skeleton)
{
    if (!( bim_skeleton.joints_.size() == bim_skeleton.parents_.size() && bim_skeleton.transforms_.size() == bim_skeleton.joints_.size() ))
        return false;

    size_t i;
    Joint* joint;
    Mat4f lcl_transform;

    if (sizeof(Mat4f) != 16*sizeof(float))
    {
        std::cerr << "bim_skeleton_to_skeleton(): Strange size problem" << std::endl;
        return false;
    }

    for (i=0; i < bim_skeleton.joints_.size(); ++i)
    {
        if (bim_skeleton.transforms_[i].size() == 16)
        {
            memcpy(lcl_transform.data(), &bim_skeleton.transforms_[i][0], sizeof(Mat4f));
        }
        else
        {
            lcl_transform = Mat4f::identity();
            std::cerr << "bim_skeleton_to_skeleton(): Transform has wrong size" << std::endl;
        }

        joint = new Joint(bim_skeleton.joints_[i].c_str(), lcl_transform);

        skeleton.joints_.push_back(joint);
    }

    for (i=0; i < bim_skeleton.joints_.size(); ++i)
    {
        joint = skeleton.joints_[i];
        joint->parent_ = skeleton.get_joint(bim_skeleton.parents_[i].c_str());
        if (joint->parent_ != NULL)
        {
            joint->parent_->children_.push_back(joint);
        }
        else
        {
            skeleton.root_ = joint;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------

bool bim_mesh_to_blendshapetarget(const bim::BIM_file::BIM_mesh& bim_mesh, Surface_mesh* base, Surface_mesh* target)
{
    std::vector<unsigned int> moving_indices;
    std::vector<Point>        moving_vertices;
    std::vector<Normal>       moving_normals;

    std::map<std::string, std::string>::const_iterator m_it;


    //copy/convert vertices (vec3)
    m_it = bim_mesh.semantic_to_data_.find("float3vec blendshape_movingvertices");
    if (m_it != bim_mesh.semantic_to_data_.end() && Point::size() == 3)
    {
        memcpy_vecT(moving_vertices, m_it->second);
    }

    //copy/convert vertices (vec4)
    m_it = bim_mesh.semantic_to_data_.find("float4vec blendshape_movingvertices");
    if (m_it != bim_mesh.semantic_to_data_.end() && Point::size() == 4)
    {
        memcpy_vecT(moving_vertices, m_it->second);
    }

    if (moving_vertices.empty())
    {
        std::cerr << "IO_binary_character: bim_mesh_to_blendshapetarget() [ERROR] Got no moving vertices for blendshape target." << std::endl;
        return false;
    }


    //copy/convert indices
    m_it = bim_mesh.semantic_to_data_.find("uintvec blendshape_movingindices");
    if (m_it != bim_mesh.semantic_to_data_.end())
    {
        memcpy_vecT(moving_indices, m_it->second);
    }

    if (moving_indices.empty())
    {
        std::cerr << "IO_binary_character: bim_mesh_to_blendshapetarget() [ERROR] Got no indices for moving vertices for blendshape target." << std::endl;
        return false;
    }
    else if (moving_indices.size() != moving_vertices.size())
    {
        std::cerr << "IO_binary_character: bim_mesh_to_blendshapetarget() [ERROR] Moving indices and vertices have different size." << std::endl;
        return false;
    }


    //copy/convert normals
    m_it = bim_mesh.semantic_to_data_.find("float3vec blendshape_movingnormals");
    if (m_it != bim_mesh.semantic_to_data_.end())
    {
        memcpy_vecT(moving_normals, m_it->second);
    }


    Surface_mesh::Vertex_property<Point>  base_vpoint  = base->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Normal> base_vnormal = base->get_vertex_property<Normal>("v:normal");


    if (!base_vnormal || !base_vpoint)
    {
        std::cerr << "IO_binary_character: bim_mesh_to_blendshapetarget() [ERROR] Base mesh has no vertices/normals?!?" << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<Point>  target_vpoint  = target->vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Normal> target_vnormal = target->vertex_property<Normal>("v:normal");

    //copy everything from base
    Surface_mesh::Vertex_iterator vit;
    for (vit = base->vertices_begin(); vit != base->vertices_end(); ++vit)
    {
        target->add_vertex(base_vpoint[*vit]);
        target_vnormal[*vit] = base_vnormal[*vit];
    }

    //replace vertices and normals at moving indices position
    for (size_t v=0; v < moving_indices.size(); ++v)
    {
        target_vpoint[Surface_mesh::Vertex((int)moving_indices[v])] = moving_vertices[v];

        if (!moving_normals.empty())
        {
            target_vnormal[Surface_mesh::Vertex((int)moving_indices[v])] = moving_normals[v];
        }
    }

    return true;
}

//-----------------------------------------------------------------------------

bool bim_blendshapes_to_blendshapes(bim::BIM_file& bim, Character *character)
{
    Blendshapes::Target_shape* target;
    std::map<std::string, bim::BIM_file::BIM_mesh>::const_iterator m_it;
    Surface_mesh::Mesh_property<std::string> m_id;

    for (size_t k = 0; k < bim.blendshapes_.size(); ++k)
    {
        const bim::BIM_file::BIM_blendshapes& bim_bs = bim.blendshapes_[k];

        Blendshapes* new_bs = new Blendshapes;

        //find base shape in meshes
        m_it = bim.meshes_.find(bim_bs.base_id_);

        if (m_it != bim.meshes_.end())
        {
            Surface_mesh_skin* base;
            base = new Surface_mesh_skin;
            //convert base shape data to Surface mesh and init if successful
            if (bim_mesh_to_mesh(m_it->second, base, bim.filename_))
            {
                m_id = base->mesh_property<std::string>("m:id");
                m_id[0] = m_it->first;
                base->init_properties();
                new_bs->set_base(base);
                character->skins().push_back(base);
            }
            else // clear if not
            {
                new_bs->set_base(NULL);
                delete base;
            }
            //erase iterator from map to not load it as a skin afterwards
            bim.meshes_.erase(m_it);
        }

        if (new_bs->base() == NULL)
        {
            delete new_bs;
            continue;
        }

        for (size_t i=0; i < bim_bs.target_ids_.size(); ++i)
        {
            //find target shape data in meshes
            m_it = bim.meshes_.find(bim_bs.target_ids_[i]);
            if (m_it != bim.meshes_.end())
            {
                //create new target
                target = new Blendshapes::Target_shape;
                //set id/name of this target mesh
                m_id = target->mesh_property<std::string>("m:id");
                m_id[0] = m_it->first;

                bool success = false;

                const bim::BIM_file::BIM_mesh& bim_mesh = m_it->second;
                //check for new blendshape standard
                if (bim_mesh.semantic_to_data_.find("uintvec blendshape_movingindices") != bim_mesh.semantic_to_data_.end())
                {
                    success = bim_mesh_to_blendshapetarget(m_it->second, new_bs->base(), target);
                }
                //otherwise load like a normal mesh
                else
                {
                    success = bim_mesh_to_mesh(m_it->second, target, bim.filename_);
                }

                if (success)
                {
                    //add to blendshapes
                    new_bs->add_target(target);
                }
                else // delete if not successful
                {
                    std::cout << "IO_binary_character: bim_blendshapes_to_blendshapes [ERROR] Could not load blendshape target with ID \"" << m_id[0] << "\"." << std::endl;
                    delete target;
                }

                //erase iterator from map to not load it as a skin afterwards
                bim.meshes_.erase(m_it);
            }
        }

        if (!new_bs->targets().empty())
            character->blendshapes().push_back(new_bs);
        else
            delete new_bs;
    }


    return true;

}

//-----------------------------------------------------------------------------

bool read_binary_character(Character *character, const char *filename)
{
    bim::BIM_file bim;
    Surface_mesh_skin* skin;
    std::map<std::string, bim::BIM_file::BIM_mesh>::const_iterator m_it;
    Surface_mesh::Mesh_property<std::string> m_id;

    if (! bim.read(filename))
        return false;

    //convert skeleton
    if (!bim_skeleton_to_skeleton(bim.skeleton_, character->skeleton()))
        return false;

    if (!bim_blendshapes_to_blendshapes(bim, character))
    {
        std::cout << "IO_binary_character: load() [WARNING] Could not load Blendshapes." << std::endl;
    }


    for (m_it=bim.meshes_.cbegin(); m_it != bim.meshes_.cend(); ++m_it)
    {

        skin = new Surface_mesh_skin;

        m_id = skin->mesh_property<std::string>("m:id");
        m_id[0] = m_it->first;

        if (bim_mesh_to_mesh(m_it->second, skin, filename))
        {
            skin->init_properties();
            character->skins().push_back(skin);
        }
        else
        {
            delete skin;
        }
    }



    if (character->skins().empty() && character->skeleton().joints_.empty())
        return false;

    return true;

}

//-----------------------------------------------------------------------------

void mesh_to_bim(const Surface_mesh* mesh, bim::BIM_file& bim)
{
    std::vector<Normal> tmp_h_normals;
    std::vector<Texture_coordinate> tmp_h_texcoords;
    std::vector<unsigned int> indices;

    Surface_mesh::Vertex_property<Point> vertices_;
    Surface_mesh::Vertex_property<Color> v_colors_;
    Surface_mesh::Vertex_property<Normal> v_normals_;
    Surface_mesh::Vertex_property<Vec4f> v_weights_;
    Surface_mesh::Vertex_property<Vec4f> v_depends_;
    Surface_mesh::Vertex_property<Vec4f> v_weights2_;
    Surface_mesh::Vertex_property<Vec4f> v_depends2_;
    Surface_mesh::Vertex_property<Vec3f> v_cor_;

    Surface_mesh::Halfedge_property<Normal> h_normals_;
    Surface_mesh::Halfedge_property<Texture_coordinate> h_texcoords_;

    Surface_mesh::Mesh_property<std::string> m_texture_name;
    Surface_mesh::Mesh_property<std::string> m_texture_name_nm;
    Surface_mesh::Mesh_property<std::string> m_texture_name_spec;
    Surface_mesh::Mesh_property<std::string> m_id;
    Surface_mesh::Mesh_property<Vec3f>       m_ambient_color;
    Surface_mesh::Mesh_property<Vec3f>       m_diffuse_color;
    Surface_mesh::Mesh_property<Vec4f>       m_specular_color;
    Surface_mesh::Mesh_property<float>       m_alpha;

    Surface_mesh::Face_iterator fit;
    Surface_mesh::Vertex_around_face_circulator vfc,vfc_end;
    Surface_mesh::Halfedge_around_face_circulator hfc, hfc_end;

    std::string mesh_id = "no_name";

    vertices_ = mesh->get_vertex_property<Point>("v:point");
    v_colors_ = mesh->get_vertex_property<Color>("v:color");
    v_normals_ = mesh->get_vertex_property<Normal>("v:normal");
    v_weights_ = mesh->get_vertex_property<Vec4f>("v:skin_weight");
    v_depends_ = mesh->get_vertex_property<Vec4f>("v:skin_depend");
    v_weights2_ = mesh->get_vertex_property<Vec4f>("v:skin_weight2");
    v_depends2_ = mesh->get_vertex_property<Vec4f>("v:skin_depend2");
    v_cor_     = mesh->get_vertex_property<Vec3f>("v:cor");

    h_normals_ = mesh->get_halfedge_property<Normal>("h:normal");
    h_texcoords_ = mesh->get_halfedge_property<Texture_coordinate>("h:texcoord");

    m_texture_name      = mesh->get_mesh_property<std::string>("m:texturename");
    m_texture_name_nm   = mesh->get_mesh_property<std::string>("m:texturename_nm");
    m_texture_name_spec = mesh->get_mesh_property<std::string>("m:texturename_spec");
    m_id             = mesh->get_mesh_property<std::string>("m:id");
    m_ambient_color  = mesh->get_mesh_property<Vec3f>("m:ambient_color");
    m_diffuse_color  = mesh->get_mesh_property<Vec3f>("m:diffuse_color");
    m_specular_color = mesh->get_mesh_property<Vec4f>("m:specular_color");
    m_alpha          = mesh->get_mesh_property<float>("m:alpha");

    if (m_id)
    {
        mesh_id = m_id[0];
    }

    if (m_texture_name)
    {
        std::string fname = m_texture_name[0];
        size_t pos = fname.rfind('/');
        if (pos == std::string::npos)
            pos = fname.rfind('\\');

        fname = fname.substr(pos+1);

        bim.add_mesh_data(mesh_id, "string diffuse_texture", fname.size(), fname.c_str());
    }

    if (m_texture_name_nm)
    {
        std::string fname = m_texture_name_nm[0];
        size_t pos = fname.rfind('/');
        if (pos == std::string::npos)
            pos = fname.rfind('\\');

        fname = fname.substr(pos+1);

        bim.add_mesh_data(mesh_id, "string normalmap_texture", fname.size(), fname.c_str());
    }

    if (m_texture_name_spec)
    {
        std::string fname = m_texture_name_spec[0];
        size_t pos = fname.rfind('/');
        if (pos == std::string::npos)
            pos = fname.rfind('\\');

        fname = fname.substr(pos+1);

        bim.add_mesh_data(mesh_id, "string specularmap_texture", fname.size(), fname.c_str());
    }

    if (m_ambient_color)
    {
        bim.add_mesh_data(mesh_id, "float3 ambient_color", sizeof(Vec3f), m_ambient_color[0].data());
    }

    if (m_diffuse_color)
    {
        bim.add_mesh_data(mesh_id, "float3 diffuse_color", sizeof(Vec3f), m_diffuse_color[0].data());
    }

    if (m_specular_color)
    {
        bim.add_mesh_data(mesh_id, "float4 specular_color", sizeof(Vec4f), m_specular_color[0].data());
    }

    if (m_alpha)
    {
        bim.add_mesh_data(mesh_id, "float alpha", sizeof(float), &m_alpha[0]);
    }

    if (vertices_)
    {
        if (Point::size() == 3)
        {
            bim.add_mesh_data(mesh_id, "float3vec vertices", vertices_.vector().size()*sizeof(Point), &vertices_.vector()[0]);
        }
        else if (Point::size() == 4)
        {
            bim.add_mesh_data(mesh_id, "float4vec vertices", vertices_.vector().size()*sizeof(Point), &vertices_.vector()[0]);
        }
    }

    if (v_normals_)
    {
        bim.add_mesh_data(mesh_id, "float3vec v_normals", v_normals_.vector().size()*sizeof(Normal), &v_normals_.vector()[0]);
    }

    if (v_colors_)
    {
        if (Color::size() == 3)
        {
            bim.add_mesh_data(mesh_id, "float3vec v_colors", v_colors_.vector().size()*sizeof(Color), &v_colors_.vector()[0]);
        }
        else if (Color::size() == 4)
        {
            bim.add_mesh_data(mesh_id, "float4vec v_colors", v_colors_.vector().size()*sizeof(Color), &v_colors_.vector()[0]);
        }
    }

    if (v_depends_)
    {
        bim.add_mesh_data(mesh_id, "float4vec v_depends", v_depends_.vector().size()*sizeof(Vec4f), &v_depends_.vector()[0]);
    }

    if (v_weights_)
    {
        bim.add_mesh_data(mesh_id, "float4vec v_weights", v_weights_.vector().size()*sizeof(Vec4f), &v_weights_.vector()[0]);
    }

    if (v_depends2_)
    {
        bim.add_mesh_data(mesh_id, "float4vec v_depends2", v_depends2_.vector().size()*sizeof(Vec4f), &v_depends2_.vector()[0]);
    }

    if (v_weights2_)
    {
        bim.add_mesh_data(mesh_id, "float4vec v_weights2", v_weights2_.vector().size()*sizeof(Vec4f), &v_weights2_.vector()[0]);
    }

    if (v_cor_)
    {
        bim.add_mesh_data(mesh_id, "float3vec v_cor", v_cor_.vector().size()*sizeof(Vec3f), &v_cor_.vector()[0]);
    }

    if (h_normals_)
    {
        tmp_h_normals.clear();
        tmp_h_normals.reserve(mesh->n_faces()*3);
        for (fit = mesh->faces_begin(); fit != mesh->faces_end(); ++fit)
        {
            hfc = hfc_end = mesh->halfedges(*fit);
            do
            {
                tmp_h_normals.push_back(h_normals_[*hfc]);
            }
            while(++hfc != hfc_end);
        }
        if (Normal::size() == 3)
            bim.add_mesh_data(mesh_id, "float3vec fv_normals", tmp_h_normals.size()*sizeof(Normal), &tmp_h_normals[0]);
        else if (Normal::size() == 4)
            bim.add_mesh_data(mesh_id, "float4vec fv_normals", tmp_h_normals.size()*sizeof(Normal), &tmp_h_normals[0]);
    }

    if (h_texcoords_)
    {
        tmp_h_texcoords.clear();
        tmp_h_texcoords.reserve(mesh->n_faces()*3);
        for (fit = mesh->faces_begin(); fit != mesh->faces_end(); ++fit)
        {
            hfc = hfc_end = mesh->halfedges(*fit);
            do
            {
                tmp_h_texcoords.push_back(h_texcoords_[*hfc]);
            }
            while(++hfc != hfc_end);
        }
        if (Texture_coordinate::size() == 2)
            bim.add_mesh_data(mesh_id, "float2vec fv_texcoords", tmp_h_texcoords.size()*sizeof(Texture_coordinate), &tmp_h_texcoords[0]);
        else if (Texture_coordinate::size() == 3)
            bim.add_mesh_data(mesh_id, "float3vec fv_texcoords", tmp_h_texcoords.size()*sizeof(Texture_coordinate), &tmp_h_texcoords[0]);
    }

    if (mesh->n_faces() > 0)
    {
        indices.clear();
        indices.reserve(mesh->n_faces()*3);
        for (fit = mesh->faces_begin(); fit != mesh->faces_end(); ++fit)
        {
            vfc = vfc_end = mesh->vertices(*fit);
            do
            {
                indices.push_back((unsigned int)(*vfc).idx());
            }
            while(++vfc != vfc_end);
        }
        bim.add_mesh_data(mesh_id, "uintvec indices", indices.size()*sizeof(unsigned int), &indices[0]);
    }
}


//-----------------------------------------------------------------------------

bool blendshapetarget_to_bim(const Surface_mesh_skin* base, Blendshapes::Target_shape* target, bim::BIM_file& bim, const float moving_threshold = 1e-7f)
{
    std::vector<unsigned int> moving_indices;
    std::vector<Point>        moving_vertices;
    std::vector<Normal>       moving_normals;
    moving_indices.reserve(base->n_vertices()/4);
    moving_vertices.reserve(base->n_vertices()/4);
    moving_normals.reserve(base->n_vertices()/4);

    Surface_mesh::Vertex_property<Point>  base_vertices = base->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point>  target_vertices = target->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Normal> target_normals = target->get_vertex_property<Normal>("v:normal");

    if (!base_vertices || !target_vertices)
    {
        return false;
    }

    if (base_vertices.vector().size() != target_vertices.vector().size())
    {
        return false;
    }

    for (size_t i=0; i < base_vertices.vector().size(); ++i)
    {
        const Point &tp = target_vertices.vector()[i];
        const Point &bp = base_vertices.vector()[i];

        if (distance(tp, bp) > moving_threshold)
        {
            moving_indices.push_back((unsigned int) i);
            moving_vertices.push_back(tp);
            moving_normals.push_back(target_normals.vector()[i]);
        }
    }

    if (moving_indices.empty())
    {
        return false;
    }

    std::string mesh_id;
    Surface_mesh::Mesh_property<std::string> m_id = target->get_mesh_property<std::string>("m:id");
    if (m_id)
    {
        mesh_id = m_id[0];
    }
    else
    {
        mesh_id = "no_name";
    }

    bim.add_mesh_data(mesh_id, "uintvec blendshape_movingindices", moving_indices.size()*sizeof(unsigned int), &moving_indices[0]);

    if (Point::size() == 3)
    {
        bim.add_mesh_data(mesh_id, "float3vec blendshape_movingvertices", moving_vertices.size()*sizeof(Point), &moving_vertices[0]);
    }
    else if (Point::size() == 4)
    {
        bim.add_mesh_data(mesh_id, "float4vec blendshape_movingvertices", moving_vertices.size()*sizeof(Point), &moving_vertices[0]);
    }

    bim.add_mesh_data(mesh_id, "float3vec blendshape_movingvnormals", moving_normals.size()*sizeof(Normal), &moving_normals[0]);

    return true;
}

//-----------------------------------------------------------------------------


bool write_binary_character(const Character *character, const char *filename)
{

    size_t i;
    bim::BIM_file bim;



    //skeleton data
    Joint* joint;

    bim.skeleton_.joints_.resize(character->skeleton().joints_.size());
    bim.skeleton_.parents_.resize(character->skeleton().joints_.size());
    bim.skeleton_.transforms_.resize(character->skeleton().joints_.size(), std::vector<float>(16));

    for (i=0; i < character->skeleton().joints_.size(); ++i)
    {
        joint = character->skeleton().joints_[i];

        bim.skeleton_.joints_[i] = joint->name_;
        if (joint->parent_ != NULL)
            bim.skeleton_.parents_[i] = joint->parent_->name_;
        else
            bim.skeleton_.parents_[i] = "root";

        memcpy(&(bim.skeleton_.transforms_[i][0]), joint->bind_pose_local_.data(), 16*sizeof(float));
    }


    //skin data
    Surface_mesh_skin* skin;

    for (i=0; i < character->skins().size(); ++i)
    {
        skin = character->skins()[i];
        mesh_to_bim(skin, bim);
    }


    //blendshape data
    size_t k;
    for (k = 0; k < character->blendshapes().size(); ++k)
    {
        Blendshapes::Target_shape* target;
        bim::BIM_file::BIM_blendshapes bim_blendshapes;

        Surface_mesh::Mesh_property<std::string> m_id;

        if (character->blendshapes()[k]->base() != NULL)
        {
            m_id = character->blendshapes()[k]->base()->get_mesh_property<std::string>("m:id");

            if (m_id && character->blendshapes()[k]->base()->n_vertices() > 0)
            {
                mesh_to_bim(character->blendshapes()[k]->base(), bim);
                bim_blendshapes.base_id_ = m_id[0];

                for (i=0; i < character->blendshapes()[k]->targets().size(); ++i)
                {
                    target = character->blendshapes()[k]->targets()[i];
                    m_id = target->get_mesh_property<std::string>("m:id");

                    if (m_id)
                    {
                        bim_blendshapes.target_ids_.push_back(m_id[0]);

                        blendshapetarget_to_bim(character->blendshapes()[k]->base(), target, bim);

                        // uncomment this function to write out blendshape in the old style
                        //mesh_to_bim(target, bim);
                    }
                }

                bim.blendshapes_.push_back(bim_blendshapes);
            }
        }
    }


    return bim.write(filename);

}

//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
