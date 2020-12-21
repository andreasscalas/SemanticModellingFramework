//=============================================================================

#include "IO.h"

#include <graphene/character/data_structure/Character.h>

#include <graphene/character/data_structure/DAE_loader.h>
#include <graphene/geometry/Matrix3x3.h>

//=============================================================================

using namespace graphene::surface_mesh;

namespace graphene {
namespace character {

//=============================================================================


bool dae_mesh_to_surface_mesh(DAE_loader::DAE_Mesh* dae_mesh, Surface_mesh* s_mesh)
{
    size_t i;
    int idx_step;
    unsigned int vtx_count;

    //mesh properties
    Surface_mesh::Vertex_property<Point> vertices;
    Surface_mesh::Vertex_property<Normal> v_normals;
    Surface_mesh::Halfedge_property<Texture_coordinate> h_texcoords;
    Surface_mesh::Halfedge_property<Normal> h_normals;

    Surface_mesh::Vertex v;
    Surface_mesh::Face f;

    Surface_mesh::Halfedge_around_face_circulator h_fit,h_end;

    std::vector<Surface_mesh::Vertex> f_vertices;

    idx_step = dae_mesh->index_step_ + 1;

    //get per vertex defined data
    if (dae_mesh->vertex_offset_ != -1 && dae_mesh->vertex_stride_ > 2)
    {
        vertices = s_mesh->vertex_property<Point>("v:point");

        if (dae_mesh->normal_offset_ == -1 && dae_mesh->normal_stride_ > 2)
        {
            v_normals = s_mesh->vertex_property<Normal>("v:normal");
        }

        vtx_count = dae_mesh->vertices_.size() / dae_mesh->vertex_stride_;

        for (i=0; i < vtx_count; ++i)
        {
            v = s_mesh->add_vertex(Point(
                                         dae_mesh->vertices_[i*dae_mesh->vertex_stride_ + 0],
                                         dae_mesh->vertices_[i*dae_mesh->vertex_stride_ + 1],
                                         dae_mesh->vertices_[i*dae_mesh->vertex_stride_ + 2]
                                         ));


            if (v_normals)
            {
                v_normals[v] = Normal(
                            dae_mesh->normals_[i*dae_mesh->normal_stride_ + 0],
                            dae_mesh->normals_[i*dae_mesh->normal_stride_ + 1],
                            dae_mesh->normals_[i*dae_mesh->normal_stride_ + 2]
                            );

                v_normals[v] = normalize(v_normals[v]);
            }
        }
    }
    else
    {
        //no vertices defined
        return false;
    }


    if (dae_mesh->texcoord_offset_ != -1 && dae_mesh->texcoord_stride_ > 1)
    {
        h_texcoords = s_mesh->halfedge_property<Texture_coordinate>("h:texcoord");
    }

    if (dae_mesh->normal_offset_ != -1 && dae_mesh->normal_stride_ > 2)
    {
        h_normals = s_mesh->halfedge_property<Texture_coordinate>("h:normal");
    }



    for (i=0; i < dae_mesh->indices_.size(); i += 3*idx_step)
    {
        f_vertices.clear();
        f_vertices.push_back( Surface_mesh::Vertex(dae_mesh->indices_[i + dae_mesh->vertex_offset_ + 0*idx_step]) );
        f_vertices.push_back( Surface_mesh::Vertex(dae_mesh->indices_[i + dae_mesh->vertex_offset_ + 1*idx_step]) );
        f_vertices.push_back( Surface_mesh::Vertex(dae_mesh->indices_[i + dae_mesh->vertex_offset_ + 2*idx_step]) );

        f = s_mesh->add_face(f_vertices);
        if (f.is_valid())
        {
            if (h_texcoords)
            {
                h_fit = s_mesh->halfedges(f);
                h_end = h_fit;
                unsigned v_idx =0;
                do
                {
                    h_texcoords[*h_fit]=Texture_coordinate(
                                dae_mesh->texcoords_[dae_mesh->indices_[i + dae_mesh->texcoord_offset_ + v_idx*idx_step] * dae_mesh->texcoord_stride_ + 0],
                                dae_mesh->texcoords_[dae_mesh->indices_[i + dae_mesh->texcoord_offset_ + v_idx*idx_step] * dae_mesh->texcoord_stride_ + 1],
                                0.0f
                            );
                    ++v_idx;
                    ++h_fit;
                }
                while(h_fit!=h_end);
            }

            if (h_normals)
            {
                h_fit = s_mesh->halfedges(f);
                h_end = h_fit;
                unsigned v_idx =0;
                do
                {
                    h_normals[*h_fit] = Normal(
                                        dae_mesh->normals_[dae_mesh->indices_[i + dae_mesh->normal_offset_ + v_idx*idx_step]*dae_mesh->normal_stride_ + 0],
                                        dae_mesh->normals_[dae_mesh->indices_[i + dae_mesh->normal_offset_ + v_idx*idx_step]*dae_mesh->normal_stride_ + 1],
                                        dae_mesh->normals_[dae_mesh->indices_[i + dae_mesh->normal_offset_ + v_idx*idx_step]*dae_mesh->normal_stride_ + 2]
                                    );

                    h_normals[*h_fit] = normalize(h_normals[*h_fit]);

                    ++v_idx;
                    ++h_fit;
                }
                while(h_fit!=h_end);
            }
        }
    }

    if (!h_normals && !v_normals)
    {
        s_mesh->update_face_normals();
        s_mesh->update_vertex_normals();
    }


    if (s_mesh->n_vertices() != 0)
    {
        //transform mesh (points and normals) with first instance of dae_mesh (if any)
        if (!dae_mesh->instances_.empty())
        {
            const Mat4f instance = dae_mesh->instances_[0];
            const Mat3f instance_it = inverse(transpose(Mat3f(instance)));

            Surface_mesh::Vertex_iterator v_it;

            for (v_it = s_mesh->vertices_begin(); v_it != s_mesh->vertices_end(); ++v_it)
            {
                s_mesh->position(*v_it) = affine_transform(instance, s_mesh->position(*v_it));

                if (v_normals)
                {
                    v_normals[*v_it] = instance_it * v_normals[*v_it];
                }
            }
            if (h_normals)
            {
                Surface_mesh::Halfedge_iterator h_it;
                for (h_it = s_mesh->halfedges_begin(); h_it != s_mesh->halfedges_end(); ++h_it)
                {
                    h_normals[*h_it] = instance_it * h_normals[*h_it];
                }
            }
        }

        //add texture filename if effect is present
        if (dae_mesh->effect_ != NULL)
        {
            if (! dae_mesh->effect_->texture_filename_.empty()) // albedo map
            {
                Surface_mesh::Mesh_property<std::string> tex_name = s_mesh->mesh_property<std::string>("m:texturename");
                tex_name[0] = dae_mesh->effect_->texture_filename_;
            }
            if (! dae_mesh->effect_->texture_filename_normalmap_.empty()) // normal map
            {
                Surface_mesh::Mesh_property<std::string> tex_name_nm = s_mesh->mesh_property<std::string>("m:texturename_nm");
                tex_name_nm[0] = dae_mesh->effect_->texture_filename_normalmap_;
            }
            if (! dae_mesh->effect_->texture_filename_specmap_.empty()) // specular map
            {
                Surface_mesh::Mesh_property<std::string> tex_name_spec = s_mesh->mesh_property<std::string>("m:texturename_spec");
                tex_name_spec[0] = dae_mesh->effect_->texture_filename_specmap_;
            }

            surface_mesh::Surface_mesh::Mesh_property<Vec3f> m_ambient_color;
            surface_mesh::Surface_mesh::Mesh_property<Vec3f> m_diffuse_color;
            surface_mesh::Surface_mesh::Mesh_property<Vec4f> m_specular_color;

            m_ambient_color  = s_mesh->mesh_property<Vec3f>("m:ambient_color");
            m_diffuse_color  = s_mesh->mesh_property<Vec3f>("m:diffuse_color");
            m_specular_color = s_mesh->mesh_property<Vec4f>("m:specular_color");

            m_ambient_color[0] = dae_mesh->effect_->ambient_color_;
            m_diffuse_color[0] = dae_mesh->effect_->diffuse_color_;
            m_specular_color[0] = dae_mesh->effect_->specular_color_;
        }


        Surface_mesh::Mesh_property<std::string> mesh_id = s_mesh->mesh_property<std::string>("m:id");
        mesh_id[0] = dae_mesh->id_;

        return true;
    }
    else
    {
        return false;
    }
}

//-----------------------------------------------------------------------------

// helper function to convert a DAE_mesh to a Surface_mesh(_skin) and then add weights and dependencies for skinning
bool dae_mesh_and_dae_skin_to_surface_mesh_skin(DAE_loader::DAE_Mesh* dae_mesh, DAE_loader::DAE_Skin* dae_skin, Surface_mesh_skin* sm_skin, Skeleton& skeleton)
{
    if (!dae_mesh_to_surface_mesh(dae_mesh, sm_skin))
        return false;

    sm_skin->vertex_property<Vec4f>("v:skin_weight");
    sm_skin->vertex_property<Vec4f>("v:skin_depend");
    sm_skin->vertex_property<Vec4f>("v:skin_weight2");
    sm_skin->vertex_property<Vec4f>("v:skin_depend2");

    sm_skin->init_properties();

    size_t j;
    int idx,vc,k,offset_step,depend,v_idx;
    float r;

    std::string jnt_name;
    Vec4f tmp_depend1, tmp_weight1, tmp_depend2, tmp_weight2;

    std::vector< std::pair<float, int> > pairs_weight_depend(4);
    std::pair<float, int> temp_pair;

    //get depend values and weights from v and vcount arrays
    idx = 0;
    offset_step = dae_skin->index_step_+1;
    for (j=0; j < dae_skin->vcount_.size(); ++j)
    {

        pairs_weight_depend.clear();
        pairs_weight_depend.resize(8, std::pair<float, int>(0.0f, 0));

        vc = dae_skin->vcount_[j];
        tmp_depend1 = tmp_depend2 = Vec4f(0.0f);
        tmp_weight1 = tmp_weight2 = Vec4f(0.0f);

        for (k=0; k < vc; ++k)
        {
            v_idx = dae_skin->v_[idx+(k*offset_step)+dae_skin->joints_offset_];
            if (v_idx != -1)
            {
                jnt_name = dae_skin->joints_array_[v_idx];
                depend = skeleton.get_joint_idx(jnt_name.c_str());
                if (depend != -1)
                {
                    //get corresponding weight
                    v_idx = dae_skin->v_[idx+(k*offset_step)+dae_skin->weights_offset_];

                    temp_pair.first = dae_skin->weights_array_[v_idx];
                    temp_pair.second = depend;

                    // find smallest weight in vector
                    int min_idx = -1;
                    float min_weight = FLT_MAX;
                    for (size_t i = 0; i < pairs_weight_depend.size(); ++i)
                    {
                        const std::pair< float, int >& p = pairs_weight_depend[i];
                        if (p.first < min_weight)
                        {
                            min_weight = p.first;
                            min_idx = i;
                        }
                    }

                    // overwrite smallest weight in vector
                    if ( temp_pair.first > min_weight )
                    {
                        pairs_weight_depend[min_idx] = temp_pair;
                    }
                }
            }
        }
        tmp_depend1 = Vec4f((float) pairs_weight_depend[0].second,
                           (float) pairs_weight_depend[1].second,
                           (float) pairs_weight_depend[2].second,
                           (float) pairs_weight_depend[3].second);

        tmp_weight1 = Vec4f(pairs_weight_depend[0].first,
                           pairs_weight_depend[1].first,
                           pairs_weight_depend[2].first,
                           pairs_weight_depend[3].first);

        tmp_depend2 = Vec4f((float) pairs_weight_depend[4].second,
                           (float) pairs_weight_depend[5].second,
                           (float) pairs_weight_depend[6].second,
                           (float) pairs_weight_depend[7].second);

        tmp_weight2 = Vec4f(pairs_weight_depend[4].first,
                           pairs_weight_depend[5].first,
                           pairs_weight_depend[6].first,
                           pairs_weight_depend[7].first);

        //normalize weights to one
        r = 1.0f/(tmp_weight1[0]+tmp_weight1[1]+tmp_weight1[2]+tmp_weight1[3] + tmp_weight2[0]+tmp_weight2[1]+tmp_weight2[2]+tmp_weight2[3]);

        sm_skin->v_depends1_[Surface_mesh::Vertex((int)j)] = tmp_depend1;
        sm_skin->v_weights1_[Surface_mesh::Vertex((int)j)] = tmp_weight1 * r;
        sm_skin->v_depends2_[Surface_mesh::Vertex((int)j)] = tmp_depend2;
        sm_skin->v_weights2_[Surface_mesh::Vertex((int)j)] = tmp_weight2 * r;


        idx += vc*(offset_step);
    }

    return true;
}

//-----------------------------------------------------------------------------


bool read_dae_character(Character* character, const char* filename)
{
    DAE_loader dae;
    if(!dae.load(filename))
        return false;

    if (dae.skeletons_.empty())
    {
        //MAYBE: load only meshes here
        return false;
    }


    size_t i;
    DAE_loader::DAE_Skeleton* dae_skeleton;
    DAE_loader::DAE_Joint* dae_joint;
    DAE_loader::DAE_Skin* dae_skin;
    DAE_loader::DAE_Mesh* dae_mesh;
    DAE_loader::DAE_Morph* dae_morph;

    std::map<std::string, DAE_loader::DAE_Skin*>::iterator skin_it;

    Joint* new_joint;
    Surface_mesh_skin* new_skin;

    //for now: use only first skeleton
    dae_skeleton = dae.skeletons_[0];

    //construct skeleton
    for (i=0; i < dae_skeleton->joints_.size(); ++i)
    {
        dae_joint = dae_skeleton->joints_[i];
        new_joint = new Joint(dae_joint->name_.c_str(), dae_joint->transform_);
        if (dae_joint->parent_) //if parent, add current joint to children of parent
        {
            new_joint->parent_ = character->skeleton().get_joint(dae_joint->parent_->name_.c_str());
            new_joint->parent_->children_.push_back(new_joint);
        }
        else // if no parent, joint is the root joint
        {
            character->skeleton().root_ = new_joint;
        }

        character->skeleton().joints_.push_back(new_joint);
    }

    if (character->skeleton().root_ == NULL)
    {
        std::cerr << "read_dae_character: skeleton without root -> aborting" << std::endl;
        return false;
    }



    //iterate through skins and create skins
    for (skin_it = dae.skin_lib_.begin(); skin_it != dae.skin_lib_.end(); ++skin_it)
    {
        dae_skin = skin_it->second;

        for (i=0; i < dae_skin->joints_array_.size(); ++i)
        {
            const std::string& jnt_name = dae_skin->joints_array_[i];
            if (character->skeleton().get_joint_idx(jnt_name.c_str()) == -1)
                break;
        }

        if (i == dae_skin->joints_array_.size())
        {
            dae_mesh = NULL;
            if (dae.mesh_lib_.count(dae_skin->target_id_) > 0)
            {
                dae_mesh = dae.mesh_lib_[dae_skin->target_id_];
                dae_mesh->instances_ = dae_skin->instances_;
                dae_mesh->effect_ = dae_skin->effect_;

                new_skin = new Surface_mesh_skin;

                if (dae_mesh_and_dae_skin_to_surface_mesh_skin(dae_mesh, dae_skin, new_skin, character->skeleton()))
                {
                    character->skins().push_back(new_skin);
                }
                else
                {
                    delete new_skin;
                }
            }
            else if (dae.morph_lib_.count(dae_skin->target_id_) > 0)
            {
                dae_morph = dae.morph_lib_[dae_skin->target_id_];
                if (dae.mesh_lib_.count(dae_morph->base_mesh_id_) > 0)
                {
                    dae_mesh = dae.mesh_lib_[dae_morph->base_mesh_id_];
                    dae_mesh->instances_ = dae_skin->instances_;
                    dae_mesh->effect_ = dae_skin->effect_;

                    new_skin = new Surface_mesh_skin;

                    if (dae_mesh_and_dae_skin_to_surface_mesh_skin(dae_mesh, dae_skin, new_skin, character->skeleton()))
                    {
                        Blendshapes* new_bs = new Blendshapes;
                        new_bs->set_base(new_skin);
                        character->skins().push_back(new_skin);
                        //load blendshapes
                        for (i=0; i < dae_morph->target_ids_.size(); ++i)
                        {
                            if (dae.mesh_lib_.count(dae_morph->target_ids_[i]) > 0)
                            {
                                dae_mesh = dae.mesh_lib_[dae_morph->target_ids_[i]];

                                Blendshapes::Target_shape* target = new Blendshapes::Target_shape();

                                if (dae_mesh->instances_.empty())
                                {
                                    dae_mesh->instances_.push_back(dae.global_transform_);
                                }

                                if (dae_mesh_to_surface_mesh(dae_mesh, target))
                                {
                                    new_bs->add_target(target);
                                }
                                else
                                {
                                    delete target;
                                }
                            }
                        }
                        character->blendshapes().push_back(new_bs);
                    }
                    else
                    {
                        delete new_skin;
                    }

                }
            }
        }
        else
        {
            std::cerr << "read_dae_character: Could not find skeleton for skin \"" << dae_skin->id_ << "\"" << std::endl;
        }
    }


    return true;
}

//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
