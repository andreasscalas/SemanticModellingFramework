//=============================================================================
#ifndef GRAPHENE_TEETH_CORRECTION_HELPER_H
#define GRAPHENE_TEETH_CORRECTION_HELPER_H
//=============================================================================

//== INCLUDES =================================================================

#include <graphene/character/data_structure/Character.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace teeth_correction_helper {



bool correct_teeth(character::Character& inout_character,
                   const std::string& s_bodymesh,
                   const std::string& s_teeth_up,
                   const std::string& s_teeth_down,
                   const std::string &fn_sel_mouth_region);


//=============================================================================
} // namespace teeth_helper
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

