//=============================================================================

#include <graphene/character/data_structure/Retargeting.h>

#include <fstream>

//=============================================================================

namespace graphene {
namespace character {
//=============================================================================

bool Retargeting::convert_skeleton(const std::string &fn_skel_conv)
{
    std::set<std::string> to_delete;
    std::map<std::string, std::string> to_rename;

    if (! read_skeleton_conversion(fn_skel_conv, to_delete, to_rename))
    {
        std::cerr << "Retargeting::convert_skeleton: [ERROR] Unable to read file with conversion information: \"" << fn_skel_conv << "\"." << std::endl;
        return false;
    }


    std::set<std::string>::const_iterator smit;
    for (smit = to_delete.begin(); smit != to_delete.end(); ++smit)
    {
        character_.delete_joint(*smit);
    }

    std::map<std::string, std::string>::const_iterator cmit;
    for (cmit = to_rename.begin(); cmit != to_rename.end(); ++cmit)
    {
        Joint* j = character_.skeleton().get_joint(cmit->first.c_str());
        if (j != nullptr)
        {
            j->name_ = cmit->second;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------


bool Retargeting::apply_retargeting(const std::string &fn_skeleton, const std::string &excluded_joints)
{

    std::set<std::string> exclude_list;
    if (!read_excluded_joints(excluded_joints, exclude_list))
    {
        std::cerr << "Retargeting::apply_retargeting: [ERROR] Unable to read excluded joints." << std::endl;
        return false;
    }



    Character icspace_character;
    if (! icspace_character.read(fn_skeleton.c_str()))
    {
        std::cerr << "Retargeting::apply_retargeting: [ERROR] Unable to read reference skeleton from file \"" << fn_skeleton << "\"." << std::endl;
        return false;
    }
    icspace_character.set_name("ICSPACE_character");
    Skeleton& icspace_skeleton = icspace_character.skeleton();


    Skeleton& local_skeleton = character_.skeleton();
    local_skeleton.align_axes();

    // Check if icspace joints are in local character. If not: collapse
    std::vector<std::string> to_collapse;
    for (size_t j=0; j < icspace_skeleton.joints_.size(); ++j)
    {
        std::string joint_name = icspace_skeleton.joints_.at(j)->name_;
        const int idx_current_joint = local_skeleton.get_joint_idx(joint_name.c_str());
        if (idx_current_joint < 0)
        {
            to_collapse.push_back(joint_name);
        }
    }
    for (size_t j=0; j < to_collapse.size(); ++j)
    {
        icspace_character.delete_joint(to_collapse[j]);
    }

    // Check if local character's joints are in icspace character. If not: error
    bool err = false;
    for (size_t j=0; j < local_skeleton.joints_.size(); ++j)
    {
        const std::string& joint_name = local_skeleton.joints_[j]->name_;
        const int idx_current_joint   = icspace_skeleton.get_joint_idx(joint_name.c_str());
        if (idx_current_joint < 0)
        {
            err = true;
            std::cerr << "Retargeting::apply_retargeting: [ERROR] Found invalid joint: " << joint_name << std::endl;
        }
    }

    if (err)
    {
        std::cerr << "Retargeting::apply_retargeting: [ERROR] Loaded character contains invalid joints. Check the cerr output." << std::endl;
        return false;
    }

    // Put the root positions at the same location:
    icspace_skeleton.root_->local_[12] = local_skeleton.root_->local_[12];
    icspace_skeleton.root_->local_[13] = local_skeleton.root_->local_[13];
    icspace_skeleton.root_->local_[14] = local_skeleton.root_->local_[14];
    icspace_skeleton.root_->update();


    // Scale ICSPACE Skeleton:
    scale_skeleton(*icspace_skeleton.root_, local_skeleton, exclude_list);
    icspace_skeleton.set_current_pose_as_bindpose();
    icspace_skeleton.update();


    adapt_joint_rotations(*icspace_skeleton.root_, local_skeleton, exclude_list);
    icspace_skeleton.set_current_pose_as_bindpose();
    copy_cs_orientations(*icspace_skeleton.root_, local_skeleton);

    return true;
}

//-----------------------------------------------------------------------------

bool Retargeting::read_skeleton_conversion(const std::string &fn_skel_conv,
                                         std::set<std::string> &to_delete,
                                         std::map<std::string, std::string> &to_rename)
{
    std::ifstream ifs;

    ifs.open(fn_skel_conv);

    if (!ifs)
        return false;

    std::string line;
    std::string tmp1,tmp2;
    std::stringstream ss;

    while (true)
    {
        std::getline(ifs, line);

        if (!ifs.good())
            break;

        if (line.empty())
            continue;

        ss.clear();
        ss.str("");
        ss << line;

        ss >> tmp1;

        if (tmp1 == "delete")
        {
            ss >> tmp1;
            to_delete.insert(tmp1);
        }
        else if (tmp1 == "rename")
        {
            ss >> tmp1;
            ss >> tmp2;
            to_rename[tmp1] = tmp2;
        }
    }

    return true;
}


bool Retargeting::read_excluded_joints(const std::string &fn_excluded_joints, std::set<std::string>& exclude_list)
{

    std::ifstream ifs(fn_excluded_joints.c_str());

    if (!ifs)
        return false;

    exclude_list.clear();

    std::string line, joint_name;
    std::stringstream ss;

    while (ifs)
    {
        std::getline(ifs, line);
        if (!ifs)
            break;

        if (line.find("#") == 0 || line.find("//") == 0)
            continue;

        ss.clear();
        ss.str(line);

        ss >> joint_name;
        //std::cerr << "[DEBUG] Exclude joint " << joint_name << std::endl;

        exclude_list.insert(joint_name);

    }
    ifs.close();
    return true;
}


//-----------------------------------------------------------------------------


void Retargeting::scale_skeleton(Joint &joint, const Skeleton &reference_skeleton, const std::set<std::string> &exclude_list)
{

    graphene::character::Joint* ref_joint = reference_skeleton.get_joint(joint.get_name().c_str());
    for (size_t c=0; c < joint.children_.size(); ++c)
    {
        graphene::character::Joint* child = joint.children_.at(c);
        graphene::character::Joint* ref_child = reference_skeleton.get_joint(child->get_name().c_str());

        Vec3f own_bone = child->get_global_translation() - joint.get_global_translation();
        Vec3f ref_bone = ref_child->get_global_translation() - ref_joint->get_global_translation();

        if (exclude_list.find(joint.name_) != exclude_list.end())
        {
            child->local_[12] = ref_child->local_[12];
            child->local_[13] = ref_child->local_[13];
            child->local_[14] = ref_child->local_[14];
        }
        else
        {
            Scalar size_ref = norm(ref_bone);
            Scalar size_own = norm(own_bone);

            Scalar scaling = size_ref / size_own;
            own_bone = own_bone * scaling;
            child->local_[12] = own_bone[0];
            child->local_[13] = own_bone[1];
            child->local_[14] = own_bone[2];
        }
        child->update();
        scale_skeleton(*child, reference_skeleton, exclude_list);
    }
    return;
}


void Retargeting::
adapt_joint_rotations(Joint& joint, const Skeleton& target_skeleton, const std::set<std::string>& exclude_list)
{
    if (joint.parent_ != 0 && !(exclude_list.find(joint.parent_->name_) != exclude_list.end()))
    {
       character::Joint* target_joint = target_skeleton.get_joint(joint.get_name().c_str());
        Vec3f vec_local = Vec3f(joint.local_[12], joint.local_[13], joint.local_[14]).normalize();
        Vec3f vec_target = Vec3f(target_joint->local_[12], target_joint->local_[13], target_joint->local_[14]).normalize();

        Mat4f modified = Mat4f();
        Mat4f vx = Mat4f();
        Vec3f v = cross(vec_local, vec_target);
        vx[0] = 0;
        vx[1] = v[2];
        vx[2] = -v[1];
        vx[3] = 0;

        vx[4] = -v[2];
        vx[5] = 0;
        vx[6] = v[0];
        vx[7] = 0;

        vx[8] = v[1];
        vx[9] = -v[0];
        vx[10] = 0;
        vx[11] = 0;

        vx[12] = 0;
        vx[13] = 0;
        vx[14] = 0;
        vx[15] = 1;

        modified = Mat4f::identity() + vx + (1/(1+dot(vec_local, vec_target))) * (vx * vx);

        modified[12] = 0;
        modified[13] = 0;
        modified[14] = 0;
        modified[15] = 1;

        joint.parent_->local_ = joint.parent_->local_ * modified;

        joint.local_[0] = inverse(modified)[0];
        joint.local_[1] = inverse(modified)[1];
        joint.local_[2] = inverse(modified)[2];
        joint.local_[3] = inverse(modified)[3];
        joint.local_[4] = inverse(modified)[4];
        joint.local_[5] = inverse(modified)[5];
        joint.local_[6] = inverse(modified)[6];
        joint.local_[7] = inverse(modified)[7];
        joint.local_[8] = inverse(modified)[8];
        joint.local_[9] = inverse(modified)[9];
        joint.local_[10] = inverse(modified)[10];
        joint.local_[11] = inverse(modified)[11];

        joint.parent_->update();

    }
    for (size_t c=0; c < joint.children_.size(); ++c)
    {
        adapt_joint_rotations(*joint.children_.at(c), target_skeleton, exclude_list);
    }
}

void Retargeting::
copy_cs_orientations(graphene::character::Joint& joint_src, graphene::character::Skeleton& target_skeleton)
{
    graphene::character::Joint *j1,*j2;

    target_skeleton.reset_to_bindpose();
    j1 = target_skeleton.get_joint(joint_src.get_name().c_str());
    j1->bind_pose_local_ = joint_src.bind_pose_local_;

    for (size_t i=0; i < j1->children_.size(); ++i)
    {
        j2 = j1->children_[i];
        j2->bind_pose_local_ = inverse(j1->bind_pose_local_) * j2->bind_pose_local_;
    }

    target_skeleton.reset_to_bindpose();
    target_skeleton.init();

    for (size_t c=0; c < joint_src.children_.size(); ++c)
    {
        copy_cs_orientations(*joint_src.children_.at(c), target_skeleton);
    }
}


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================

