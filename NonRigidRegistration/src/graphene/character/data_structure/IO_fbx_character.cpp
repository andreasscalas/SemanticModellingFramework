//=============================================================================

#include "IO.h"
#ifdef HAVE_FBX
#include <graphene/character/data_structure/fbxfile/fbx_file.h>
#endif

//=============================================================================

using namespace graphene::surface_mesh;

namespace graphene {
namespace character {

//=============================================================================


#ifdef HAVE_FBX
bool write_fbx_character(const Character *character, const char *filename)
{
    FBX_file file;
    return file.write(character, filename);
}

bool read_fbx_character(Character *character, const char *filename)
{
    FBX_file file;
    return file.read(character, filename);
}
#else

bool write_fbx_character(const Character *character, const char *filename)
{
    std::cout << "IO_fbx_character: [ERROR] Cannot write fbx character without FBX SDK. Please make sure cmake has found FBX lib and include!" << std::endl;
    return false;
}

bool read_fbx_character(Character *character, const char *filename)
{
    std::cout << "IO_fbx_character: [ERROR] Cannot read fbx character without FBX SDK. Please make sure cmake has found FBX lib and include!" << std::endl;
    return false;
}
#endif


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
