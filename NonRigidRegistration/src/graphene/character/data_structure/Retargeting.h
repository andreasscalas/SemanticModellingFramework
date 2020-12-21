//=============================================================================

#ifndef GRAPHENE_RETARGETING_H
#define GRAPHENE_RETARGETING_H

//=============================================================================

#include <graphene/character/data_structure/Character.h>

#include <vector>
#include <set>
#include <map>

//=============================================================================

namespace graphene {
namespace character {
//=============================================================================

class Retargeting
{
    Character& character_;
public:
    Retargeting(Character& ch) :
        character_(ch)
    {}

    ~Retargeting(){}


    bool convert_skeleton(const std::string &fn_skel_conv);

    bool apply_retargeting(const std::string& fn_skeleton, const std::string& excluded_joints);

private:

    bool read_skeleton_conversion(const std::string& fn_skel_conv,
                                  std::set<std::string>& to_delete,
                                  std::map<std::string, std::string> &to_rename);

    bool read_excluded_joints(const std::string& fn_excluded_joints, std::set<std::string>& exclude_list);
    void scale_skeleton(Joint& joint,
                        const Skeleton &reference_skeleton,
                        const std::set<std::string> &exclude_list);
    void adapt_joint_rotations(Joint& joint,
                               const Skeleton& target_skeleton,
                               const std::set<std::string>& exclude_list);
    void copy_cs_orientations(graphene::character::Joint& joint_src, graphene::character::Skeleton& target_skeleton);


};

//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

