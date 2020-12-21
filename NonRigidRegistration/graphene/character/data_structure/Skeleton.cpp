//=============================================================================

#include <graphene/character/data_structure/Skeleton.h>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


Skeleton::
Skeleton()
  : root_(NULL)
{
}


//-----------------------------------------------------------------------------

Skeleton
::Skeleton(const Skeleton &rhs)
{
    operator=(rhs);
}

//-----------------------------------------------------------------------------


Skeleton::
~Skeleton()
{
    size_t i;
    for (i=0; i < joints_.size(); ++i)
    {
        delete joints_[i];
    }
}


//-----------------------------------------------------------------------------

Skeleton& Skeleton::operator =(const Skeleton& rhs)
{
    if (this != &rhs)
    {
        for (size_t i=0; i < joints_.size(); ++i)
        {
            delete joints_[i];
        }
        root_ = NULL;
        skinning_matrices_.clear();

        size_t i;
        Joint* joint,*new_joint;

        joints_.resize(rhs.joints_.size());
        for (i=0; i < rhs.joints_.size(); ++i)
        {
            joint = rhs.joints_[i];
            joints_[i] = new Joint(joint->get_name().c_str(), joint->local_, joint->bind_pose_local_, joint->initial_);
        }

        for (i=0; i < rhs.joints_.size(); ++i)
        {
            joint = rhs.joints_[i];
            new_joint = joints_[i];

            if (joint->parent_)
            {
                new_joint->parent_ = get_joint(joint->parent_->get_name().c_str());
                new_joint->parent_->children_.push_back(new_joint);
            }
            else
            {
                root_ = new_joint;
            }
        }
        init();

    }

    return *this;
}

//-----------------------------------------------------------------------------

void
Skeleton::
init()
{
    skinning_matrices_.clear();
    skinning_matrices_.resize(joints_.size(), Mat4f::identity());

    root_->init();

    update();
}

//-----------------------------------------------------------------------------

void
Skeleton::
update()
{
    if (root_ == NULL)
        return;

    root_->update();

    Joint* j;
    for (unsigned int i=0; i < joints_.size(); ++i)
    {
        j = joints_[i];

        skinning_matrices_[i] = j->final_;
    }

}


//-----------------------------------------------------------------------------


void
Skeleton::set_current_pose_as_bindpose()
{
    size_t i;
    Joint* joint;

    for (i=0; i < joints_.size(); ++i)
    {
        joint = joints_[i];
        joint->bind_pose_local_ = joint->local_;
    }
}


//-----------------------------------------------------------------------------


void
Skeleton::reset_to_initial(bool keep_translation)
{
    Joint* joint;
    size_t i;

    if (keep_translation)
    {
        int m,n;
        for (i=0; i < joints_.size(); ++i)
        {
            joint = joints_[i];
            for (m=0; m < 3; ++m)
                for (n=0; n < 3; ++n)
                    joint->local_(m,n) = joint->initial_(m,n);
        }
    }
    else
    {
        for (i=0; i < joints_.size(); ++i)
        {
            joint = joints_[i];
            joint->local_ = joint->initial_;
        }
    }

    update();
}


//-----------------------------------------------------------------------------

void
Skeleton::
reset_to_bindpose()
{
    Joint* joint;
    for (size_t i=0; i < joints_.size(); ++i)
    {
        joint = joints_[i];
        joint->local_ = joint->bind_pose_local_;
    }

    update();
}

//-----------------------------------------------------------------------------


Joint*
Skeleton::
get_joint(const char *joint_name) const
{
    size_t i;
    Joint* j;
    for (i=0; i < joints_.size(); ++i)
    {
        j = joints_[i];
        if (j->name_ == joint_name)
        {
            return j;
        }
    }
    return NULL;
}

//-----------------------------------------------------------------------------


int
Skeleton::
get_joint_idx(const char *joint_name) const
{
    size_t i;
    Joint* j;
    for (i=0; i < joints_.size(); ++i)
    {
        j = joints_[i];
        if (j->name_ == joint_name)
        {
            return (int)i;
        }
    }
    return -1;
}

//-----------------------------------------------------------------------------

void
Skeleton::
align_axes()
{
    reset_to_bindpose();
    root_->init_aligned_axes();
}

//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
