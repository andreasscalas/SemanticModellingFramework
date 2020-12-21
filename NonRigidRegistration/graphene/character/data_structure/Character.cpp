//=============================================================================

#include "Character.h"

#include <graphene/geometry/Matrix3x3.h>
#include <graphene/geometry/Dual_quaternion.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/meanvalue_coords.h>

#include "IO.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <map>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


Character::Character() :
    name_("Graphene character"),
    selected_skin_idx_(0)
{

}

//-----------------------------------------------------------------------------

Character::~Character()
{
    size_t i;

    for (i=0; i < skins_.size(); ++i)
        delete skins_[i];

    for (i=0; i < blendshapes_.size(); ++i)
        delete blendshapes_[i];
}


//-----------------------------------------------------------------------------


const geometry::Bounding_box Character::bbox()
{
    geometry::Bounding_box bbox;
    size_t i;
    for (i=0; i < skins_.size(); ++i)
    {
        bbox += skins_[i]->bbox();
    }

    for (i=0; i < skeleton_.joints_.size(); ++i)
    {
        bbox += skeleton_.joints_[i]->get_global_translation();
    }

    return bbox;
}


//-----------------------------------------------------------------------------


bool
Character::
read(const char *filename)
{
    // extract file extension
    std::string fname(filename);
    std::string::size_type dot(fname.rfind("."));

    if (dot == std::string::npos)
        return false;

    std::string ext = fname.substr(dot+1, fname.length()-dot-1);
    std::transform(ext.begin(), ext.end(), ext.begin(), tolower);

    bool is_read = false;
    // extension determines reader
    if (ext == "dae")
    {
        is_read = read_dae_character(this, filename);
    }
    else if (ext == "bim")
    {
        is_read = read_binary_character(this, filename);
    }
    else if (ext == "model")
    {
        is_read = read_model_character(this, filename);
    }
    else if (ext == "fbx")
    {
        is_read = read_fbx_character(this, filename);
    }

    // update indices of moving vertices
    for (unsigned int i = 0; i < blendshapes_.size(); ++i)
    {
       Blendshapes* blendshape = blendshapes_[i];
       blendshape->update_moving_indizes();
    }


    return is_read;
}

bool
Character::
read_model_character(Character *character, const char *filename) {
    std::ifstream ifs(filename);
    if (!ifs.good()) return false;

    std::string line, jointname, parentname;
    std::stringstream ss;

    //skip first line
    std::getline(ifs,line);

    unsigned int i,j;
    Mat4f lcl_mat;

    Joint *new_joint, *parent_joint;
    Skeleton& skeleton = character->skeleton();

    while (true) {
        std::getline(ifs,line);
        if (line.find("end_skeleton") == 0)//marks end of skeleton part
        {
            break;
        }

        ss.clear();
        ss.str(line);

        //first joint name...
        ss >> jointname;
        //...then parentname
        ss >> parentname;

        //get out column major matrix
        for (i = 0; i < 4; ++i) {
            for (j = 0; j < 4; ++j) {
                ss >> lcl_mat(j,i);
            }
        }

        new_joint = new Joint(jointname.c_str(), lcl_mat);

        //look for parent joint (null if root)
        parent_joint = skeleton_.get_joint(parentname.c_str());
        if (parent_joint)
        {
            new_joint->parent_ = parent_joint;
            parent_joint->children_.push_back(new_joint);
        }
        else {
            skeleton.root_ = new_joint;
        }

        skeleton.joints_.push_back(new_joint);
        if (!ifs.good()) break;

    }

    skeleton.init();

    Surface_mesh_skin* new_skin = new Surface_mesh_skin;
    new_skin->add_vertex(Point(0.0f));
    new_skin->add_vertex(Point(0.0f));
    new_skin->add_vertex(Point(0.0f));
    new_skin->add_triangle(surface_mesh::Surface_mesh::Vertex(0),
                           surface_mesh::Surface_mesh::Vertex(1),
                           surface_mesh::Surface_mesh::Vertex(2));
    new_skin->vertex_property<Vec4f>("v:skin_weight");
    new_skin->vertex_property<Vec4f>("v:skin_depend");
    new_skin->vertex_property<Vec4f>("v:skin_weight2");
    new_skin->vertex_property<Vec4f>("v:skin_depend2");

    surface_mesh::Surface_mesh::Mesh_property<std::string> prop  = new_skin->mesh_property<std::string>("m:id");
    prop[0] = "icspace41337";
    new_skin->init_properties();

    new_skin->v_depends1_[surface_mesh::Surface_mesh::Vertex(0)] = 0;
    new_skin->v_weights1_[surface_mesh::Surface_mesh::Vertex(0)] = 0;
    new_skin->v_depends2_[surface_mesh::Surface_mesh::Vertex(1)] = 0;
    new_skin->v_weights2_[surface_mesh::Surface_mesh::Vertex(1)] = 0;

    character->skins().push_back(new_skin);


    //TODO: skin is ignored. Can be added some time later on if necessary.
    ifs.close();
    if (skeleton_.joints_.empty())
    {
        return false;
    }
    return true;
}


//-----------------------------------------------------------------------------


bool
Character::
write_bim(const char *filename) const
{
    return write_binary_character(this, filename);
}


//-----------------------------------------------------------------------------


bool
Character::
write_fbx(const char *filename) const
{
    return write_fbx_character(this, filename);
}


//-----------------------------------------------------------------------------


void
Character::
delete_skin(Surface_mesh_skin *skin)
{
    std::vector<Surface_mesh_skin*>::iterator it = std::find(
                                                    skins_.begin(),
                                                    skins_.end(),
                                                    skin);
    if (it != skins_.end())
    {
        delete *it;
        skins_.erase(it);
    }
}


//-----------------------------------------------------------------------------


void
Character::
apply_skinning_on_CPU(bool only_selected_skin, Skinning_mode mode)
{
    switch(mode)
    {
    case LINEAR_BLENDING:
        if (only_selected_skin)
            apply_linear_skinning_on_CPU_selected_skin();
        else
            apply_linear_skinning_on_CPU();
        break;
    case DUALQUAT_BLENDING:
        if (only_selected_skin)
            apply_dualquat_skinning_on_CPU_selected_skin();
        else
            apply_dualquat_skinning_on_CPU();
        break;
    }
}

void Character::apply_skinning_on_CPU(const std::vector<surface_mesh::Surface_mesh::Vertex> &indices, std::vector<Point> &result, Skinning_mode mode)
{
    switch(mode)
    {
    case LINEAR_BLENDING:
        apply_linear_skinning_on_CPU(selected_skin_idx_, indices, result);
        break;
    case DUALQUAT_BLENDING:
        apply_dualquat_skinning_on_CPU(selected_skin_idx_, indices, result);
        break;
    }
}


//-----------------------------------------------------------------------------


void Character::set_current_pose_as_bindpose()
{
    skeleton_.set_current_pose_as_bindpose();
}


//-----------------------------------------------------------------------------


void Character::correct_joint_positions()
{

    std::vector<Point> joint_positions;

    if (surface_mesh::mvc::get_points(get_selected_skin(), joint_positions, "m:mvc_joints"))
    {
        set_new_joint_positions(joint_positions);
    }
    else
    {
        std::cout << "Character::correct_joint_positions: [WARNING] No mean value coordinates present." << std::endl;
    }
}

int Character::get_skin_idx(const std::string &name)
{
    for (size_t i=0; i < skins_.size(); ++i)
    {
        surface_mesh::Surface_mesh::Mesh_property<std::string> mid = skins_[i]->get_mesh_property<std::string>("m:id");

        if (mid)
        {
            const std::string& id = mid[0];

            if (id.find(name) != std::string::npos)
            {
                return (int) i;
            }
        }
    }

    //not found
    return -1;
}


void Character::apply_linear_skinning_on_CPU()
{
    size_t i,j;
    character::Surface_mesh_skin* skin;
    Mat4f blend_mat;


    for (i=0; i < skins_.size(); ++i)
    {
        skin = skins_[i];

        const std::vector<Vec4f>& w = skin->v_weights1_.vector();
        const std::vector<Vec4f>& d = skin->v_depends1_.vector();
        const std::vector<Vec4f>& w2 = skin->v_weights2_.vector();
        const std::vector<Vec4f>& d2 = skin->v_depends2_.vector();


        for (j=0; j < skin->vertices_.vector().size(); ++j)
        {
            blend_mat = w[j][0] * skeleton_.skinning_matrices_[(int)d[j][0]];
            blend_mat+= w[j][1] * skeleton_.skinning_matrices_[(int)d[j][1]];
            blend_mat+= w[j][2] * skeleton_.skinning_matrices_[(int)d[j][2]];
            blend_mat+= w[j][3] * skeleton_.skinning_matrices_[(int)d[j][3]];

            blend_mat+= w2[j][0] * skeleton_.skinning_matrices_[(int)d2[j][0]];
            blend_mat+= w2[j][1] * skeleton_.skinning_matrices_[(int)d2[j][1]];
            blend_mat+= w2[j][2] * skeleton_.skinning_matrices_[(int)d2[j][2]];
            blend_mat+= w2[j][3] * skeleton_.skinning_matrices_[(int)d2[j][3]];

            //transform vertex and normal
            skin->vertices_.vector()[j] = affine_transform(blend_mat, skin->vertices_.vector()[j]);
            if (skin->v_normals_)
            {
                skin->v_normals_.vector()[j] = normalize(Mat3f(blend_mat) * skin->v_normals_.vector()[j]);
            }
        }
    }


    skeleton_.init();
}


void Character::apply_dualquat_skinning_on_CPU()
{
    size_t i,j;
    Joint* joint;

    std::vector<DQuatf> dqs(skeleton_.skinning_matrices_.size());

    for (i=0; i < skeleton_.skinning_matrices_.size(); ++i)
    {
        joint = skeleton_.joints_[i];
        DQuatf &dq = dqs[i];

        mat4_to_dualquat(skeleton_.skinning_matrices_[i], dq);
    }


    Mat4f blend_mat;
    DQuatf blend_dq;
    int depend,k;
    character::Surface_mesh_skin* skin;
    for (i=0; i < skins_.size(); ++i)
    {
        skin = skins_[i];

        const std::vector<Vec4f>& w = skin->v_weights1_.vector();
        const std::vector<Vec4f>& d = skin->v_depends1_.vector();
        const std::vector<Vec4f>& w2 = skin->v_weights2_.vector();
        const std::vector<Vec4f>& d2 = skin->v_depends2_.vector();

        for (j=0; j < skin->vertices_.vector().size(); ++j)
        {

            blend_dq = DQuatf(0,0,0,0,0,0,0,0);
            for (k=0; k < 4; ++k)
            {
                depend = (int)d[j][k];
                DQuatf& tmp_dq = dqs[depend];
                if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                {
                    blend_dq += tmp_dq * w[j][k];
                } else
                {
                    blend_dq -= tmp_dq * w[j][k];
                }
            }

            for (k=0; k < 4; ++k)
            {
                depend = (int)d2[j][k];
                DQuatf& tmp_dq = dqs[depend];
                if(dot(blend_dq.q0, tmp_dq.q0) > 0)
                {
                    blend_dq += tmp_dq * w2[j][k];
                } else
                {
                    blend_dq -= tmp_dq * w2[j][k];
                }
            }

            blend_dq = normalize(blend_dq);

            dq_to_mat4(blend_dq, blend_mat);

            //transform vertex and normal
            skin->vertices_.vector()[j] = affine_transform(blend_mat, skin->vertices_.vector()[j]);
            if (skin->v_normals_)
            {
                skin->v_normals_.vector()[j] = normalize(Mat3f(blend_mat) * skin->v_normals_.vector()[j]);
            }
        }
    }

    skeleton_.init();
}

void Character::apply_linear_skinning_on_CPU_selected_skin()
{
    size_t j;
    character::Surface_mesh_skin* skin;
    Mat4f blend_mat;


    skin = get_selected_skin();

    const std::vector<Vec4f>& w = skin->v_weights1_.vector();
    const std::vector<Vec4f>& d = skin->v_depends1_.vector();
    const std::vector<Vec4f>& w2 = skin->v_weights2_.vector();
    const std::vector<Vec4f>& d2 = skin->v_depends2_.vector();


    for (j=0; j < skin->vertices_.vector().size(); ++j)
    {
        blend_mat = w[j][0] * skeleton_.skinning_matrices_[(int)d[j][0]];
        blend_mat+= w[j][1] * skeleton_.skinning_matrices_[(int)d[j][1]];
        blend_mat+= w[j][2] * skeleton_.skinning_matrices_[(int)d[j][2]];
        blend_mat+= w[j][3] * skeleton_.skinning_matrices_[(int)d[j][3]];

        blend_mat+= w2[j][0] * skeleton_.skinning_matrices_[(int)d2[j][0]];
        blend_mat+= w2[j][1] * skeleton_.skinning_matrices_[(int)d2[j][1]];
        blend_mat+= w2[j][2] * skeleton_.skinning_matrices_[(int)d2[j][2]];
        blend_mat+= w2[j][3] * skeleton_.skinning_matrices_[(int)d2[j][3]];

        //transform vertex and normal
        skin->vertices_.vector()[j] = affine_transform(blend_mat, skin->vertices_.vector()[j]);
        if (skin->v_normals_)
        {
            skin->v_normals_.vector()[j] = normalize(Mat3f(blend_mat) * skin->v_normals_.vector()[j]);
        }
    }

    skeleton_.init();
}

void Character::apply_dualquat_skinning_on_CPU_selected_skin()
{
    size_t i,j;
    Joint* joint;

    std::vector<DQuatf> dqs(skeleton_.skinning_matrices_.size());

    for (i=0; i < skeleton_.skinning_matrices_.size(); ++i)
    {
        joint = skeleton_.joints_[i];
        DQuatf &dq = dqs[i];

        mat4_to_dualquat(skeleton_.skinning_matrices_[i], dq);
    }


    Mat4f blend_mat;
    DQuatf blend_dq;
    int depend,k;
    character::Surface_mesh_skin* skin;

    skin = get_selected_skin();

    const std::vector<Vec4f>& w = skin->v_weights1_.vector();
    const std::vector<Vec4f>& d = skin->v_depends1_.vector();
    const std::vector<Vec4f>& w2 = skin->v_weights2_.vector();
    const std::vector<Vec4f>& d2 = skin->v_depends2_.vector();

    for (j=0; j < skin->vertices_.vector().size(); ++j)
    {

        blend_dq = DQuatf(0,0,0,0,0,0,0,0);
        for (k=0; k < 4; ++k)
        {
            depend = (int)d[j][k];
            DQuatf& tmp_dq = dqs[depend];
            if(dot(blend_dq.q0, tmp_dq.q0) > 0)
            {
                blend_dq += tmp_dq * w[j][k];
            } else
            {
                blend_dq -= tmp_dq * w[j][k];
            }
        }

        for (k=0; k < 4; ++k)
        {
            depend = (int)d2[j][k];
            DQuatf& tmp_dq = dqs[depend];
            if(dot(blend_dq.q0, tmp_dq.q0) > 0)
            {
                blend_dq += tmp_dq * w2[j][k];
            } else
            {
                blend_dq -= tmp_dq * w2[j][k];
            }
        }

        blend_dq = normalize(blend_dq);

        dq_to_mat4(blend_dq, blend_mat);

        //transform vertex and normal
        skin->vertices_.vector()[j] = affine_transform(blend_mat, skin->vertices_.vector()[j]);
        if (skin->v_normals_)
        {
            skin->v_normals_.vector()[j] = normalize(Mat3f(blend_mat) * skin->v_normals_.vector()[j]);
        }
    }

    skeleton_.init();
}

void
Character::
apply_linear_skinning_on_CPU(const unsigned int skin_idx,
                             const std::vector<surface_mesh::Surface_mesh::Vertex> &indices,
                             std::vector<Point> &result)
{

    size_t j;

    result.resize(indices.size());


    Mat4f blend_mat;
    character::Surface_mesh_skin* skin;


    skin = skins_[skin_idx];

    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> w = skin->v_weights1_;
    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> d = skin->v_depends1_;
    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> w2 = skin->v_weights2_;
    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> d2 = skin->v_depends2_;


    for (j=0; j < indices.size(); ++j)
    {
        blend_mat = w[indices[j]][0] * skeleton_.skinning_matrices_[(int)d[indices[j]][0]];
        blend_mat+= w[indices[j]][1] * skeleton_.skinning_matrices_[(int)d[indices[j]][1]];
        blend_mat+= w[indices[j]][2] * skeleton_.skinning_matrices_[(int)d[indices[j]][2]];
        blend_mat+= w[indices[j]][3] * skeleton_.skinning_matrices_[(int)d[indices[j]][3]];

        blend_mat+= w2[indices[j]][0] * skeleton_.skinning_matrices_[(int)d2[indices[j]][0]];
        blend_mat+= w2[indices[j]][1] * skeleton_.skinning_matrices_[(int)d2[indices[j]][1]];
        blend_mat+= w2[indices[j]][2] * skeleton_.skinning_matrices_[(int)d2[indices[j]][2]];
        blend_mat+= w2[indices[j]][3] * skeleton_.skinning_matrices_[(int)d2[indices[j]][3]];

        //transform vertex and normal
        result[j] = affine_transform(blend_mat, skin->vertices_[indices[j]]);
    }
}

void Character::
apply_dualquat_skinning_on_CPU(
        const unsigned int skin_idx,
        const std::vector<surface_mesh::Surface_mesh::Vertex> &indices,
        std::vector<Point> &result)
{
    size_t i,j;
    Joint* joint;

    std::vector<DQuatf> dqs(skeleton_.skinning_matrices_.size());

    for (i=0; i < skeleton_.skinning_matrices_.size(); ++i)
    {
        joint = skeleton_.joints_[i];
        DQuatf &dq = dqs[i];

        mat4_to_dualquat(skeleton_.skinning_matrices_[i], dq);
    }

    result.resize(indices.size());


    Mat4f blend_mat;
    DQuatf blend_dq;
    int depend,k;
    character::Surface_mesh_skin* skin;


    skin = skins_[skin_idx];

    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> w = skin->v_weights1_;
    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> d = skin->v_depends1_;
    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> w2 = skin->v_weights2_;
    const surface_mesh::Surface_mesh::Vertex_property<Vec4f> d2 = skin->v_depends2_;


    for (j=0; j < indices.size(); ++j)
    {

        blend_dq = DQuatf(0,0,0,0,0,0,0,0);
        for (k=0; k < 4; ++k)
        {
            depend = (int)d[indices[j]][k];
            DQuatf& tmp_dq = dqs[depend];
            if(dot(blend_dq.q0, tmp_dq.q0) > 0)
            {
                blend_dq += tmp_dq * w[indices[j]][k];
            } else
            {
                blend_dq -= tmp_dq * w[indices[j]][k];
            }
        }

        for (k=0; k < 4; ++k)
        {
            depend = (int)d2[indices[j]][k];
            DQuatf& tmp_dq = dqs[depend];
            if(dot(blend_dq.q0, tmp_dq.q0) > 0)
            {
                blend_dq += tmp_dq * w2[indices[j]][k];
            } else
            {
                blend_dq -= tmp_dq * w2[indices[j]][k];
            }
        }

        blend_dq = normalize(blend_dq);


        dq_to_mat4(blend_dq, blend_mat);

        //transform vertex
        result[j] = affine_transform(blend_mat, skin->vertices_[indices[j]]);
    }

}


//-----------------------------------------------------------------------------


void Character::set_new_joint_positions(const std::vector<Vec3f> &positions)
{
    if (positions.size() != skeleton_.joints_.size())
    {
        return;
    }


    size_t i;
    Joint *joint;
    for (i=0; i < positions.size(); ++i)
    {
        joint = skeleton_.joints_[i];
        const Vec3f& p = positions[i];

        joint->global_[12] = p[0];
        joint->global_[13] = p[1];
        joint->global_[14] = p[2];
    }


    Vec3f o;
    for (i=0; i < positions.size(); ++i)
    {
        joint = skeleton_.joints_[i];

        if (joint->parent_)
        {
            o = joint->get_global_translation() - joint->parent_->get_global_translation();
            //transform offset into coordinate system of parent
            o = transpose(Mat3f(joint->parent_->global_)) * o;
        }
        else
        {
            o = joint->get_global_translation();
        }

        joint->local_[12] = o[0];
        joint->local_[13] = o[1];
        joint->local_[14] = o[2];
    }

    skeleton_.init();
    set_current_pose_as_bindpose();

}


//-----------------------------------------------------------------------------


bool Character::delete_joint(const std::string &joint_name)
{
    character::Joint* joint_to_delete = skeleton_.get_joint(joint_name.c_str());

    if (joint_to_delete == nullptr)
    {
        std::cout << "Character::delete_joint: [ERROR] Cannot delete joint. Joint was not found in character." << std::endl;
        return false;
    }

    //reset to bindpose in order to have a clean start
    skeleton_.reset_to_bindpose();



    if (joint_to_delete->parent_ == nullptr)
    {
        //if the joint to delete is the root, set a new root for the skeleton
        if (skeleton_.root_ == joint_to_delete)
        {
            if (! joint_to_delete->children_.empty())
                skeleton_.root_ = joint_to_delete->children_[0];
        }

        if (! joint_to_delete->children_.empty())
        {
            // use the first child of the root as the new root
            character::Joint* first_child = joint_to_delete->children_[0];
            first_child->parent_ = nullptr; // parent is the joint to delete
            for (size_t i=1; i < joint_to_delete->children_.size(); ++i)
            {
                // first child is now the new parent of other children of the root
                first_child->children_.push_back(joint_to_delete->children_[i]);
                joint_to_delete->children_[i]->parent_ = first_child;
                //multiply the inverse of the new root to the local transform of its new children
                joint_to_delete->children_[i]->local_  = inverse(first_child->local_) * joint_to_delete->children_[i]->local_;
            }
            //make sure the old transformation of the root is taken into account
            first_child->local_ =  joint_to_delete->local_ *first_child->local_ ;
        }
    }
    else // joint is not root
    {
        Joint* parent = joint_to_delete->parent_;
        //delete joint from parent's children list
        for (size_t i=0; i < parent->children_.size(); ++i)
        {
            if (parent->children_[i] == joint_to_delete)
            {
                parent->children_.erase(parent->children_.begin() + i);
                break;
            }
        }

        //handle children of the joint to delete
        for (size_t i=0; i < joint_to_delete->children_.size(); ++i)
        {
            //reparent children to parent of the joint to delete
            joint_to_delete->children_[i]->parent_ = joint_to_delete->parent_;
            joint_to_delete->parent_->children_.push_back(joint_to_delete->children_[i]);

            //make sure the transformation of the joint to delete is preserved
            joint_to_delete->children_[i]->local_  = joint_to_delete->local_ * joint_to_delete->children_[i]->local_;
        }
    }

    //delete the joint itself and from the joint vector
    const int joint_to_del_idx = skeleton_.get_joint_idx(joint_name.c_str());
    skeleton_.joints_.erase(skeleton_.joints_.begin() + joint_to_del_idx);

    //get parent pointer first
    Joint* parent  = joint_to_delete->parent_;
    delete joint_to_delete;


    //correct the dependencies of the mesh's vertices

    int parent_idx = 0;
    if (parent != nullptr)
    {
        parent_idx = skeleton_.get_joint_idx(parent->get_name().c_str());
    }

    if (parent_idx == -1)
        parent_idx = 0;


    std::map<int,float> d2w;

    for (size_t i=0; i < skins_.size(); ++i)
    {
        character::Surface_mesh_skin* skin = skins_[i];

        for (size_t j=0; j < skin->v_depends1_.vector().size(); ++j)
        {
            d2w.clear();
            Vec4f &d1 = skin->v_depends1_.vector()[j];
            Vec4f &w1 = skin->v_weights1_.vector()[j];
            Vec4f &d2 = skin->v_depends2_.vector()[j];
            Vec4f &w2 = skin->v_weights2_.vector()[j];

            for (int k=0; k < 4; ++k)
            {
                std::map<int,float>::iterator fit = d2w.find((int)d1[k]);
                if (fit != d2w.end())
                {
                    fit->second += w1[k];
                }
                else
                {
                    d2w[(int)d1[k]] = w1[k];
                }
            }

            for (int k=0; k < 4; ++k)
            {
                std::map<int,float>::iterator fit = d2w.find((int)d2[k]);
                if (fit != d2w.end())
                {
                    fit->second += w2[k];
                }
                else
                {
                    d2w[(int)d2[k]] = w2[k];
                }
            }

            std::map<int,float>::iterator j2d;

            //does this vertex depend on the deleted joint?
            j2d = d2w.find(joint_to_del_idx);
            if (j2d != d2w.end()) // yes -> set to parent idx
            {
                //do we already have the parent idx in our map?
                std::map<int,float>::iterator parent = d2w.find(parent_idx);
                if (parent != d2w.end()) // yes -> just add weight
                {
                    parent->second += j2d->second;
                }
                else // no, create entry with weight
                {
                    d2w[parent_idx] = j2d->second;
                }
                // in any case: delete entry of old dependency
                d2w.erase(j2d);
            }

            //delete old dependencies and weights
            d1 = d2 = w1 = w2 = Vec4f(0.0f);

            //replace with new weights and dependencies
            int c=0;
            float dc,wc;
            std::map<int,float>::const_iterator cit;
            for (cit = d2w.cbegin(); cit != d2w.cend(); ++cit)
            {
                if (cit->second > 0.0f)
                {
                    if (cit->first > joint_to_del_idx && cit->first > 0)
                    {
                        dc = (float) (cit->first - 1);
                    }
                    else
                    {
                        dc = (float) cit->first;
                    }

                    wc = cit->second;

                    if (c < 4)
                    {
                        d1[c] = dc;
                        w1[c] = wc;
                    }
                    else
                    {
                        d2[c-4] = dc;
                        w2[c-4] = wc;
                    }

                    ++c;
                }
            }
        }
    }

    //init skeleton's matrices
    skeleton_.init();
    //set new bindpose
    skeleton_.set_current_pose_as_bindpose();
}


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
