//=============================================================================
#ifndef GRAPHENE_EYES_CORRECTION_HELPER_H
#define GRAPHENE_EYES_CORRECTION_HELPER_H
//=============================================================================

//== INCLUDES =================================================================

#include <graphene/character/data_structure/Character.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace eyes_correction_helper {

struct Eye_info
{
    std::string fn_sel_eyeregion_left;
    std::string fn_sel_eyeregion_right;

    std::string s_eyemesh_left;
    std::string s_eyemesh_right;
    std::string s_eyetrans_left;
    std::string s_eyetrans_right;
    std::string s_eyegland_left;
    std::string s_eyegland_right;
    std::string s_eyejoint_left;
    std::string s_eyejoint_right;
    std::string s_bodymesh;
};

/// correct eyes
/// this is basically a reimplementation of update_eyes_ver2 in Eyes_helper.h
bool correct_eyes(character::Character& inout_character,
                  const Eye_info& eye_info);


//=============================================================================
} // namespace eyes_correction_helper
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

