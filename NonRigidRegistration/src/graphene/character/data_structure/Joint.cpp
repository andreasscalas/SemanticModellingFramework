//=============================================================================

#include <graphene/character/data_structure/Joint.h>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


Joint::
Joint(const char *name, const Mat4f &local) :
    name_(name),
    parent_(NULL),
    initial_(local),
    bind_pose_local_(local),
    local_(local),
    global_inv_(Mat4f::identity()),
    global_(Mat4f::identity()),
    final_(Mat4f::identity())
{}


//-----------------------------------------------------------------------------


Joint::
Joint(const char *name, const Mat4f &local, const Mat4f& bind_pose_local, const Mat4f &initial) :
    name_(name),
    parent_(NULL),
    initial_(initial),
    bind_pose_local_(bind_pose_local),
    local_(local),
    global_inv_(Mat4f::identity()),
    global_(Mat4f::identity()),
    final_(Mat4f::identity())
{}


//-----------------------------------------------------------------------------


Joint::
~Joint()
{
}


//-----------------------------------------------------------------------------


void
Joint::
init()
{
    if (parent_) {
        global_ = parent_->global_ * local_;
    }
    else {
        global_ = local_;
    }
    global_inv_ = inverse(global_);
    final_ = global_ * global_inv_;

    Joint* joint;
    for (size_t i=0; i < children_.size(); ++i)
    {
        joint = children_[i];
        joint->init();
    }
}

//-----------------------------------------------------------------------------

void Joint::init_aligned_axes()
{
    local_ = Mat4f::identity();

    const Vec3f pos = get_global_translation();

    Vec3f offset_;
    if (parent_)
    {
        offset_ = pos - parent_->get_global_translation();
    }
    else
    {
        offset_ = pos;
    }
    local_(0,3) = offset_[0];
    local_(1,3) = offset_[1];
    local_(2,3) = offset_[2];
    local_(3,3) = 1.0f;

    bind_pose_local_ = local_;

    if (parent_)
    {
        global_ = parent_->global_ * local_;
    }
    else
    {
        global_ = local_;
    }
    global_inv_ = inverse(global_);


    Joint* joint;
    for (unsigned int i=0; i < children_.size(); ++i)
    {
        joint = children_[i];
        joint->init_aligned_axes();
    }
}

//-----------------------------------------------------------------------------

void Joint::update()
{
    if (parent_)
    {
        global_ = parent_->global_
                * local_;
    }
    else
    {
        global_ = local_;
    }
    final_ = global_ * global_inv_;

    Joint* joint;
    for (unsigned int i=0; i < children_.size(); ++i)
    {
        joint = children_[i];
        joint->update();
    }
}

//-----------------------------------------------------------------------------

void Joint::print()
{
    if (parent_)
    {
        std::cout << "\"" << parent_->name_.c_str() << "\" is parent of ";
    }
    else
    {
        std::cout << "root is ";
    }

    std::cout << "\"" << name_.c_str() << "\"" << std::endl;

    Joint* joint;
    for (unsigned int i=0; i < children_.size(); ++i)
    {
        joint = children_[i];
        joint->print();
    }
}

//-----------------------------------------------------------------------------

const Vec3f Joint::get_global_translation()
{
    Vec3f result;
    result[0] = global_[12];
    result[1] = global_[13];
    result[2] = global_[14];
    return result;
}

//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
