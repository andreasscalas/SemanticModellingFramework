//=============================================================================

#ifndef GRAPHENE_SKELETON_H
#define GRAPHENE_SKELETON_H

//=============================================================================

#include <graphene/character/data_structure/Joint.h>

#include <vector>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================

///enum for skinning mode
///currently only used for switching CPU skinning mode
enum Skinning_mode
{
    LINEAR_BLENDING = 0,
    DUALQUAT_BLENDING
};

class Skeleton
{
public:
    /// The root joint
    Joint* root_;

    /// joint pointers
    std::vector<Joint*> joints_;

    /// the skinning matrices that are uploaded to skinning shader
    std::vector<Mat4f> skinning_matrices_;


public:

    Skeleton();
    Skeleton(const Skeleton& s);
    ~Skeleton();

    Skeleton &operator =(const Skeleton& rhs);

    /// initialize joints
    void init();

    /// update forward kinematic with local axes matrices (see Joint struct)
    void update();

    ///reset skeleton to initial/loaded matrices
    void reset_to_initial(bool keep_translation = false);

    ///set the current pose described by local matrices as the bind pose
    void set_current_pose_as_bindpose();

    ///reset skeleton to bindpose
    void reset_to_bindpose();

    /// get a specific joint by name
    Joint* get_joint(const char* joint_name) const;

    /// get index of joint by name
    int get_joint_idx(const char* joint_name) const;

    /// align all axes of all joints with the global axes. This cannot be undone!
    void align_axes();
};


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
