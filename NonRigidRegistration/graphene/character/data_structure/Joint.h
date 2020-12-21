//=============================================================================

#ifndef GRAPHENE_JOINT_H
#define GRAPHENE_JOINT_H

//=============================================================================

#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Matrix4x4.h>

#include <vector>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


class Joint
{
public:
    std::string name_;

    Joint* parent_;

    std::vector<Joint*> children_;

    //the very initial matrix (as loaded; never changed, thus const)
    const Mat4f initial_;

    //the bind pose matrix (may changes due to new bind pose)
    Mat4f bind_pose_local_;

    //matrix for rotating around local axis
    Mat4f local_; //full joint-to-joint trafo at init
    Mat4f global_inv_; //full joint back trafo

    //the global transform computed by update()
    Mat4f global_;
    //the current final matrix (global_*global_inv_), which is then given to skinning (shader),
    Mat4f final_;

public:

    Joint(const char* name, const Mat4f& local);
    Joint(const char* name, const Mat4f& local, const Mat4f& bind_pose_local, const Mat4f& initial);
    ~Joint();

    ///initialize local matrix and compute inverse matrix
    void init();
    ///initialize local matrix for aligned axes and corresponding inverse matrix (only works with preceded call of init())
    void init_aligned_axes();
    ///update forward kinematic with local matrix
    void update();

    void print();

    const Vec3f get_global_translation();

    const std::string get_name() const { return name_; };

public:

};


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

