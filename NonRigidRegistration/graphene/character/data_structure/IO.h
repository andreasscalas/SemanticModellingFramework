//=============================================================================

#ifndef GRAPHENE_CHARACTER_IO_H
#define GRAPHENE_CHARACTER_IO_H

//=============================================================================

#include <graphene/character/data_structure/Character.h>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================

bool read_dae_character(Character* character, const char* filename);

bool write_binary_character(const Character *character, const char* filename);

bool read_binary_character(Character* character, const char* filename);


bool write_fbx_character(const Character* character, const char* filename);

bool read_fbx_character(Character* character, const char* filename);

//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
#endif
//=============================================================================

