//== INCLUDES ===================================================================


#include "Character_generator.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>
#include <graphene/surface_mesh/eigen_algorithms/deformation_transfer/Deformation_transfer_botsch.h>

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/Triangle_kD_tree.h>

#include <graphene/surface_mesh/eigen_algorithms/rbf_deformer/RBF_deformer.h>

#include <graphene/geometry/registration.h>

#include <graphene/geometry/distance_point_triangle.h>

#include <graphene/surface_mesh/algorithms/subdivision/loop_subdivision.h>
#include <graphene/surface_mesh/algorithms/texture_processing/Texture_processing.h>
#include <graphene/surface_mesh/algorithms/texture_processing/utility/my_helper.h>

#include <graphene/surface_mesh/algorithms/templatefitting/Template_fit.h>

#include <opencv2/opencv.hpp>


#include <fstream> // TODO DEBUGGING ONLY


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ============================================================


Character_generator::
Character_generator(const std::string dir_database)
    : dir_database_(dir_database), character_node_(NULL), latest_hairstyle_type_(Hairstyle_keep)
{
    init();
}


//-----------------------------------------------------------------------------


Character_generator::
Character_generator()
    : character_node_(NULL), latest_hairstyle_type_(Hairstyle_keep)
{
    init();
}


//-----------------------------------------------------------------------------


void
Character_generator::
init()
{
    // mapping model type to filename
    map_modeltype_2_filename_[Model_custom_M_young]    = "00_custom/CG_M_young_178cm_Ver4/CG_M_young_178cm_Ver4.bim";
    map_modeltype_2_filename_[Model_custom_W_young]    = "00_custom/CG_W_young_178cm_Ver3/CG_W_young_178cm_Ver3.bim";
    map_modeltype_2_filename_[Model_custom_M_midyoung] = "00_custom/CG_M_midyoung_178cm_Ver4/CG_M_midyoung_178cm_Ver4.bim";
    map_modeltype_2_filename_[Model_custom_W_midyoung] = "00_custom/CG_W_midyoung_178cm_Ver4/CG_W_midyoung_178cm_Ver4.bim";
    map_modeltype_2_filename_[Model_custom_M_midold]   = "00_custom/CG_M_midold_178cm_Ver2/CG_M_midold_178cm_Ver2.bim";
    map_modeltype_2_filename_[Model_custom_W_midold]   = "00_custom/CG_W_midold_178cm_Ver3/CG_W_midold_178cm_Ver3.bim";
    map_modeltype_2_filename_[Model_custom_M_old]      = "00_custom/CG_M_old_178cm_Ver3/CG_M_old_178cm_Ver3.bim";
    map_modeltype_2_filename_[Model_custom_W_old]      = "00_custom/CG_W_old_178cm_Ver3/CG_W_old_178cm_Ver3.bim";
    map_modeltype_2_filename_[Model_M_Anthony]         = "CG_M_Anthony/CG_M_Anthony.bim";
    map_modeltype_2_filename_[Model_M_Asaiah]          = "CG_M_Asaiah/CG_M_Asaiah.bim";
    map_modeltype_2_filename_[Model_M_Biff]            = "CG_M_Biff/CG_M_Biff.bim";
    map_modeltype_2_filename_[Model_M_Buck]            = "CG_M_Buck/CG_M_Buck.bim";
    map_modeltype_2_filename_[Model_M_Cody]            = "CG_M_Cody/CG_M_Cody.bim";
    map_modeltype_2_filename_[Model_M_David]           = "CG_M_David/CG_M_David.bim";
    map_modeltype_2_filename_[Model_M_Dylan]           = "CG_M_Dylan/CG_M_Dylan.bim";
    map_modeltype_2_filename_[Model_M_Elias]           = "CG_M_Elias/CG_M_Elias.bim";
    map_modeltype_2_filename_[Model_M_Evan]            = "CG_M_Evan/CG_M_Evan.bim";
    map_modeltype_2_filename_[Model_M_Gabriel]         = "CG_M_Gabriel/CG_M_Gabriel.bim";
    map_modeltype_2_filename_[Model_M_Gilbert]         = "CG_M_Gilbert/CG_M_Gilbert.bim";
    map_modeltype_2_filename_[Model_M_Huey]            = "CG_M_Huey/CG_M_Huey.bim";
    map_modeltype_2_filename_[Model_M_Jack]            = "CG_M_Jack/CG_M_Jack.bim";
    map_modeltype_2_filename_[Model_M_Jacob]           = "CG_M_Jacob/CG_M_Jacob.bim";
    map_modeltype_2_filename_[Model_M_Jamal]           = "CG_M_Jamal/CG_M_Jamal.bim";
    map_modeltype_2_filename_[Model_M_JeanRene]        = "CG_M_JeanRene/CG_M_JeanRene.bim";
    map_modeltype_2_filename_[Model_M_Jeremyah]        = "CG_M_Jeremyah/CG_M_Jeremyah.bim";
    map_modeltype_2_filename_[Model_M_Johnny]          = "CG_M_Johnny/CG_M_Johnny.bim";
    map_modeltype_2_filename_[Model_M_Jose]            = "CG_M_Jose/CG_M_Jose.bim";
    map_modeltype_2_filename_[Model_M_Kevin]           = "CG_M_Kevin/CG_M_Kevin.bim";
    map_modeltype_2_filename_[Model_M_Kurt]            = "CG_M_Kurt/CG_M_Kurt.bim";
    map_modeltype_2_filename_[Model_M_Luke]            = "CG_M_Luke/CG_M_Luke.bim";
    map_modeltype_2_filename_[Model_M_Maurice]         = "CG_M_Maurice/CG_M_Maurice.bim";
    map_modeltype_2_filename_[Model_M_Minh]            = "CG_M_Minh/CG_M_Minh.bim";
    map_modeltype_2_filename_[Model_M_Nathan]          = "CG_M_Nathan/CG_M_Nathan.bim";
    map_modeltype_2_filename_[Model_M_Ralph]           = "CG_M_Ralph/CG_M_Ralph.bim";
    map_modeltype_2_filename_[Model_M_Ryan]            = "CG_M_Ryan/CG_M_Ryan.bim";
    map_modeltype_2_filename_[Model_M_Schwartz]        = "CG_M_Schwartz/CG_M_Schwartz.bim";
    map_modeltype_2_filename_[Model_M_Sean]            = "CG_M_Sean/CG_M_Sean.bim";
    map_modeltype_2_filename_[Model_M_Stephan]         = "CG_M_Stephan/CG_M_Stephan";
    map_modeltype_2_filename_[Model_M_Taylor]          = "CG_M_Taylor/CG_M_Taylor.bim";
    map_modeltype_2_filename_[Model_M_Terence]         = "CG_M_Terence/CG_M_Terence.bim";
    map_modeltype_2_filename_[Model_M_Timothy]         = "CG_M_Timothy/CG_M_Timothy.bim";
    map_modeltype_2_filename_[Model_M_Todd]            = "CG_M_Todd/CG_M_Todd.bim";
    map_modeltype_2_filename_[Model_M_Victor]          = "CG_M_Victor/CG_M_Victor.bim";
    map_modeltype_2_filename_[Model_M_Warren]          = "CG_M_Warren/CG_M_Warren.bim";
    map_modeltype_2_filename_[Model_M_William]         = "CG_M_William/CG_M_William.bim";
    map_modeltype_2_filename_[Model_M_Wyatt]           = "CG_M_Wyatt/CG_M_Wyatt.bim";
    map_modeltype_2_filename_[Model_W_Adrienne]        = "CG_W_Adrienne/CG_W_Adrienne.bim";
    map_modeltype_2_filename_[Model_W_Amy]             = "CG_W_Amy/CG_W_Amy.bim";
    map_modeltype_2_filename_[Model_W_Ashley]          = "CG_W_Ashley/CG_W_Ashley.bim";
    map_modeltype_2_filename_[Model_W_Barbara]         = "CG_W_Barbara/CG_W_Barbara.bim";
    map_modeltype_2_filename_[Model_W_Brooke]          = "CG_W_Brooke/CG_W_Brooke.bim";
    map_modeltype_2_filename_[Model_W_Cathy]           = "CG_W_Cathy/CG_W_Cathy.bim";
    map_modeltype_2_filename_[Model_W_Charley]         = "CG_W_Charley/CG_W_Charley.bim";
    map_modeltype_2_filename_[Model_W_Chloe]           = "CG_W_Chloe/CG_W_Chloe.bim";
    map_modeltype_2_filename_[Model_W_Claire]          = "CG_W_Claire/CG_W_Claire.bim";
    map_modeltype_2_filename_[Model_W_Claudia]         = "CG_W_Claudia/CG_W_Claudia.bim";
    map_modeltype_2_filename_[Model_W_Daniela]         = "CG_W_Daniela/CG_W_Daniela.bim";
    map_modeltype_2_filename_[Model_W_Debbie]          = "CG_W_Debbie/CG_W_Debbie.bim";
    map_modeltype_2_filename_[Model_W_Eleanor]         = "CG_W_Eleanor/CG_W_Eleanor.bim";
    map_modeltype_2_filename_[Model_W_Elsa]            = "CG_W_Elsa/CG_W_Elsa.bim";
    map_modeltype_2_filename_[Model_W_Esmeralda]       = "CG_W_Esmeralda/CG_W_Esmeralda.bim";
    map_modeltype_2_filename_[Model_W_Evelyn]          = "CG_W_Evelyn/CG_W_Evelyn.bim";
    map_modeltype_2_filename_[Model_W_Grace]           = "CG_W_Grace/CG_W_Grace.bim";
    map_modeltype_2_filename_[Model_W_Haley]           = "CG_W_Haley/CG_W_Haley.bim";
    map_modeltype_2_filename_[Model_W_Jada]            = "CG_W_Jada/CG_W_Jada.bim";
    map_modeltype_2_filename_[Model_W_Jeanne]          = "CG_W_Jeanne/CG_W_Jeanne.bim";
    map_modeltype_2_filename_[Model_W_Jenna]           = "CG_W_Jenna/CG_W_Jenna.bim";
    map_modeltype_2_filename_[Model_W_Kari]            = "CG_W_Kari/CG_W_Kari.bim";
    map_modeltype_2_filename_[Model_W_Kate]            = "CG_W_Kate/CG_W_Kate.bim";
    map_modeltype_2_filename_[Model_W_Katherine]       = "CG_W_Katherine/CG_W_Katherine.bim";
    map_modeltype_2_filename_[Model_W_Kim]             = "CG_W_Kim/CG_W_Kim.bim";
    map_modeltype_2_filename_[Model_W_Lena]            = "CG_W_Lena/CG_W_Lena.bim";
    map_modeltype_2_filename_[Model_W_Lily]            = "CG_W_Lily/CG_W_Lily.bim";
    map_modeltype_2_filename_[Model_W_Makayla]         = "CG_W_Makayla/CG_W_Makayla.bim";
    map_modeltype_2_filename_[Model_W_Maya]            = "CG_W_Maya/CG_W_Maya.bim";
    map_modeltype_2_filename_[Model_W_Mia]             = "CG_W_Mia/CG_W_Mia.bim";
    map_modeltype_2_filename_[Model_W_Morgan]          = "CG_W_Morgan/CG_W_Morgan.bim";
    map_modeltype_2_filename_[Model_W_Nicole]          = "CG_W_Nicole/CG_W_Nicole.bim";
    map_modeltype_2_filename_[Model_W_Paige]           = "CG_W_Paige/CG_W_Paige.bim";
    map_modeltype_2_filename_[Model_W_Pam]             = "CG_W_Pam/CG_W_Pam.bim";
    map_modeltype_2_filename_[Model_W_Rebecca]         = "CG_W_Rebecca/CG_W_Rebecca.bim";
    map_modeltype_2_filename_[Model_W_Silke]           = "CG_W_Silke/CG_W_Silke.bim";
    map_modeltype_2_filename_[Model_W_Sophia]          = "CG_W_Sophia/CG_W_Sophia.bim";
    map_modeltype_2_filename_[Model_W_Tiffany]         = "CG_W_Tiffany/CG_W_Tiffany.bim";
    map_modeltype_2_filename_[Model_W_Trinity]         = "CG_W_Trinity/CG_W_Trinity.bim";
    map_modeltype_2_filename_[Model_W_Venus]           = "CG_W_Venus/CG_W_Venus.bim";
    map_modeltype_2_filename_[Model_W_Wanda]           = "CG_W_Wanda/CG_W_Wanda.bim";
    map_modeltype_2_filename_[Model_W_Yoki]            = "CG_W_Yoki/CG_W_Yoki.bim";
    map_modeltype_2_filename_[Model_W_Zoe]             = "CG_W_Zoe/CG_W_Zoe.bim";
    map_modeltype_2_filename_[Model_W_Zyra]            = "CG_W_Zyra/CG_W_Zyra.bim";


    // mapping skin color type to filename
    map_skintype_2_filename_[Skin_M_01] = "CG_Skin_M_01/CG_Skin_M_01";
    map_skintype_2_filename_[Skin_M_02] = "CG_Skin_M_02/CG_Skin_M_02";
    map_skintype_2_filename_[Skin_M_03] = "CG_Skin_M_03/CG_Skin_M_03";
    map_skintype_2_filename_[Skin_M_04] = "CG_Skin_M_04/CG_Skin_M_04";
    map_skintype_2_filename_[Skin_M_05] = "CG_Skin_M_05/CG_Skin_M_05";
    map_skintype_2_filename_[Skin_M_06] = "CG_Skin_M_06/CG_Skin_M_06";
    map_skintype_2_filename_[Skin_M_07] = "CG_Skin_M_07/CG_Skin_M_07";
    map_skintype_2_filename_[Skin_M_08] = "CG_Skin_M_08/CG_Skin_M_08";
    map_skintype_2_filename_[Skin_M_09] = "CG_Skin_M_09/CG_Skin_M_09";
    map_skintype_2_filename_[Skin_M_10] = "CG_Skin_M_10/CG_Skin_M_10";
    map_skintype_2_filename_[Skin_M_11] = "CG_Skin_M_11/CG_Skin_M_11";
    map_skintype_2_filename_[Skin_M_12] = "CG_Skin_M_12/CG_Skin_M_12";
    map_skintype_2_filename_[Skin_M_16] = "CG_Skin_M_16/CG_Skin_M_16";
    map_skintype_2_filename_[Skin_M_17] = "CG_Skin_M_17/CG_Skin_M_17";
    map_skintype_2_filename_[Skin_M_18] = "CG_Skin_M_18/CG_Skin_M_18";
    map_skintype_2_filename_[Skin_M_19] = "CG_Skin_M_19/CG_Skin_M_19";
    map_skintype_2_filename_[Skin_M_20] = "CG_Skin_M_20/CG_Skin_M_20";
    map_skintype_2_filename_[Skin_M_21] = "CG_Skin_M_21/CG_Skin_M_21";
    map_skintype_2_filename_[Skin_M_22] = "CG_Skin_M_22/CG_Skin_M_22";
    map_skintype_2_filename_[Skin_M_23] = "CG_Skin_M_23/CG_Skin_M_23";
    map_skintype_2_filename_[Skin_M_24] = "CG_Skin_M_24/CG_Skin_M_24";
    map_skintype_2_filename_[Skin_M_25] = "CG_Skin_M_25/CG_Skin_M_25";
    map_skintype_2_filename_[Skin_M_27] = "CG_Skin_M_27/CG_Skin_M_27";
    map_skintype_2_filename_[Skin_M_28] = "CG_Skin_M_28/CG_Skin_M_28";
    map_skintype_2_filename_[Skin_M_29] = "CG_Skin_M_29/CG_Skin_M_29";
    map_skintype_2_filename_[Skin_M_30] = "CG_Skin_M_30/CG_Skin_M_30";
    map_skintype_2_filename_[Skin_M_31] = "CG_Skin_M_31/CG_Skin_M_31";
    map_skintype_2_filename_[Skin_M_32] = "CG_Skin_M_32/CG_Skin_M_32";
    map_skintype_2_filename_[Skin_M_33] = "CG_Skin_M_33/CG_Skin_M_33";
    map_skintype_2_filename_[Skin_M_36] = "CG_Skin_M_36/CG_Skin_M_36";
    map_skintype_2_filename_[Skin_M_37] = "CG_Skin_M_37/CG_Skin_M_37";
    map_skintype_2_filename_[Skin_M_38] = "CG_Skin_M_38/CG_Skin_M_38";
    map_skintype_2_filename_[Skin_M_39] = "CG_Skin_M_39/CG_Skin_M_39";
    map_skintype_2_filename_[Skin_M_40] = "CG_Skin_M_40/CG_Skin_M_40";
    map_skintype_2_filename_[Skin_M_41] = "CG_Skin_M_41/CG_Skin_M_41";
    map_skintype_2_filename_[Skin_M_44] = "CG_Skin_M_44/CG_Skin_M_44";
    map_skintype_2_filename_[Skin_M_45] = "CG_Skin_M_45/CG_Skin_M_45";
    map_skintype_2_filename_[Skin_M_46] = "CG_Skin_M_46/CG_Skin_M_46";
    map_skintype_2_filename_[Skin_M_47] = "CG_Skin_M_47/CG_Skin_M_47";
    map_skintype_2_filename_[Skin_M_48] = "CG_Skin_M_48/CG_Skin_M_48";
    map_skintype_2_filename_[Skin_M_49] = "CG_Skin_M_49/CG_Skin_M_49";
    map_skintype_2_filename_[Skin_W_01] = "CG_Skin_W_01/CG_Skin_W_01";
    map_skintype_2_filename_[Skin_W_02] = "CG_Skin_W_02/CG_Skin_W_02";
    map_skintype_2_filename_[Skin_W_03] = "CG_Skin_W_03/CG_Skin_W_03";
    map_skintype_2_filename_[Skin_W_04] = "CG_Skin_W_04/CG_Skin_W_04";
    map_skintype_2_filename_[Skin_W_05] = "CG_Skin_W_05/CG_Skin_W_05";
    map_skintype_2_filename_[Skin_W_06] = "CG_Skin_W_06/CG_Skin_W_06";
    map_skintype_2_filename_[Skin_W_07] = "CG_Skin_W_07/CG_Skin_W_07";
    map_skintype_2_filename_[Skin_W_08] = "CG_Skin_W_08/CG_Skin_W_08";
    map_skintype_2_filename_[Skin_W_09] = "CG_Skin_W_09/CG_Skin_W_09";
    map_skintype_2_filename_[Skin_W_10] = "CG_Skin_W_10/CG_Skin_W_10";
    map_skintype_2_filename_[Skin_W_11] = "CG_Skin_W_11/CG_Skin_W_11";
    map_skintype_2_filename_[Skin_W_12] = "CG_Skin_W_12/CG_Skin_W_12";
    map_skintype_2_filename_[Skin_W_13] = "CG_Skin_W_13/CG_Skin_W_13";
    map_skintype_2_filename_[Skin_W_14] = "CG_Skin_W_14/CG_Skin_W_14";
    map_skintype_2_filename_[Skin_W_15] = "CG_Skin_W_15/CG_Skin_W_15";
    map_skintype_2_filename_[Skin_W_17] = "CG_Skin_W_17/CG_Skin_W_17";
    map_skintype_2_filename_[Skin_W_19] = "CG_Skin_W_19/CG_Skin_W_19";
    map_skintype_2_filename_[Skin_W_20] = "CG_Skin_W_20/CG_Skin_W_20";
    map_skintype_2_filename_[Skin_W_21] = "CG_Skin_W_21/CG_Skin_W_21";
    map_skintype_2_filename_[Skin_W_22] = "CG_Skin_W_22/CG_Skin_W_22";
    map_skintype_2_filename_[Skin_W_23] = "CG_Skin_W_23/CG_Skin_W_23";
    map_skintype_2_filename_[Skin_W_24] = "CG_Skin_W_24/CG_Skin_W_24";
    map_skintype_2_filename_[Skin_W_25] = "CG_Skin_W_25/CG_Skin_W_25";
    map_skintype_2_filename_[Skin_W_26] = "CG_Skin_W_26/CG_Skin_W_26";
    map_skintype_2_filename_[Skin_W_27] = "CG_Skin_W_27/CG_Skin_W_27";
    map_skintype_2_filename_[Skin_W_28] = "CG_Skin_W_28/CG_Skin_W_28";
    map_skintype_2_filename_[Skin_W_29] = "CG_Skin_W_29/CG_Skin_W_29";
    map_skintype_2_filename_[Skin_W_30] = "CG_Skin_W_30/CG_Skin_W_30";
    map_skintype_2_filename_[Skin_W_31] = "CG_Skin_W_31/CG_Skin_W_31";
    map_skintype_2_filename_[Skin_W_32] = "CG_Skin_W_32/CG_Skin_W_32";
    map_skintype_2_filename_[Skin_W_33] = "CG_Skin_W_33/CG_Skin_W_33";
    map_skintype_2_filename_[Skin_W_34] = "CG_Skin_W_34/CG_Skin_W_34";
    map_skintype_2_filename_[Skin_W_35] = "CG_Skin_W_35/CG_Skin_W_35";
    map_skintype_2_filename_[Skin_W_37] = "CG_Skin_W_37/CG_Skin_W_37";
    map_skintype_2_filename_[Skin_W_38] = "CG_Skin_W_38/CG_Skin_W_38";
    map_skintype_2_filename_[Skin_W_39] = "CG_Skin_W_39/CG_Skin_W_39";
    map_skintype_2_filename_[Skin_W_40] = "CG_Skin_W_40/CG_Skin_W_40";
    map_skintype_2_filename_[Skin_W_44] = "CG_Skin_W_44/CG_Skin_W_44";
    map_skintype_2_filename_[Skin_W_45] = "CG_Skin_W_45/CG_Skin_W_45";
    map_skintype_2_filename_[Skin_W_46] = "CG_Skin_W_46/CG_Skin_W_46";
    map_skintype_2_filename_[Skin_W_47] = "CG_Skin_W_47/CG_Skin_W_47";
    map_skintype_2_filename_[Skin_W_48] = "CG_Skin_W_48/CG_Skin_W_48";
    map_skintype_2_filename_[Skin_W_49] = "CG_Skin_W_49/CG_Skin_W_49";


    // mapping eye color type to filename
    map_eyecolortype_2_filename_[Eyecolor_01] = "CG_EyeColor_01/CG_EyeColor_01_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_02] = "CG_EyeColor_02/CG_EyeColor_02_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_03] = "CG_EyeColor_03/CG_EyeColor_03_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_04] = "CG_EyeColor_04/CG_EyeColor_04_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_05] = "CG_EyeColor_05/CG_EyeColor_05_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_06] = "CG_EyeColor_06/CG_EyeColor_06_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_07] = "CG_EyeColor_07/CG_EyeColor_07_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_08] = "CG_EyeColor_08/CG_EyeColor_08_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_09] = "CG_EyeColor_09/CG_EyeColor_09_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_10] = "CG_EyeColor_10/CG_EyeColor_10_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_11] = "CG_EyeColor_11/CG_EyeColor_11_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_12] = "CG_EyeColor_12/CG_EyeColor_12_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_13] = "CG_EyeColor_13/CG_EyeColor_13_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_14] = "CG_EyeColor_14/CG_EyeColor_14_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_15] = "CG_EyeColor_15/CG_EyeColor_15_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_16] = "CG_EyeColor_16/CG_EyeColor_16_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_17] = "CG_EyeColor_17/CG_EyeColor_17_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_18] = "CG_EyeColor_18/CG_EyeColor_18_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_19] = "CG_EyeColor_19/CG_EyeColor_19_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_20] = "CG_EyeColor_20/CG_EyeColor_20_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_21] = "CG_EyeColor_21/CG_EyeColor_21_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_22] = "CG_EyeColor_22/CG_EyeColor_22_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_23] = "CG_EyeColor_23/CG_EyeColor_23_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_24] = "CG_EyeColor_24/CG_EyeColor_24_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_25] = "CG_EyeColor_25/CG_EyeColor_25_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_26] = "CG_EyeColor_26/CG_EyeColor_26_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_27] = "CG_EyeColor_27/CG_EyeColor_27_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_28] = "CG_EyeColor_28/CG_EyeColor_28_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_29] = "CG_EyeColor_29/CG_EyeColor_29_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_30] = "CG_EyeColor_30/CG_EyeColor_30_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_31] = "CG_EyeColor_31/CG_EyeColor_31_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_32] = "CG_EyeColor_32/CG_EyeColor_32_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_33] = "CG_EyeColor_33/CG_EyeColor_33_color.jpg";
    map_eyecolortype_2_filename_[Eyecolor_34] = "CG_EyeColor_34/CG_EyeColor_34_color.jpg";


    // mapping clothes type to filename
    map_clothestype_2_filename_[Clothes_custom_casual_M]                     = "CG_casual_M/CG_casual_M.bim";
    map_clothestype_2_filename_[Clothes_custom_formal_M]                     = "CG_formal_M/CG_formal_M.bim";
    map_clothestype_2_filename_[Clothes_custom_formal_W]                     = "CG_formal_W/CG_formal_W.bim";
    map_clothestype_2_filename_[Clothes_custom_sport_M]                      = "CG_sport_M/CG_sport_M.bim";
    map_clothestype_2_filename_[Clothes_custom_kitchen_M]                    = "CG_kitchen_M/CG_kitchen_M.bim";
    map_clothestype_2_filename_[Clothes_self_jascha_casual_greensuit]        = "00_self/jascha_casual_greensuit/jascha_casual_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_jascha_longsleeve_greensuit]    = "00_self/jascha_longsleeve_greensuit/jascha_longsleeve_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_jascha_tshirt_greensuit]        = "00_self/jascha_tshirt_greensuit/jascha_tshirt_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_jascha_jeansandshoes_greensuit] = "00_self/jascha_jeansandshoes_greensuit/jascha_jeansandshoes_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_teresa_casual_greensuit]        = "00_self/teresa_casual_greensuit/teresa_casual_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_teresa_jeans_greensuit]         = "00_self/teresa_jeans_greensuit/teresa_jeans_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_teresa_top_greensuit]           = "00_self/teresa_top_greensuit/teresa_top_greensuit.bim";
    map_clothestype_2_filename_[Clothes_self_christina_casual_manual]        = "00_self/christina_casual_manual/christina_casual_manual.bim";
    map_clothestype_2_filename_[Clothes_self_andreas_casual_manual]          = "00_self/andreas_casual_manual/andreas_casual_manual.bim";
    map_clothestype_2_filename_[Clothes_self_matthias_casual_manual]         = "00_self/matthias_casual_manual/matthias_casual_manual.bim";


    // mapping clothes type to mask
    map_clothestype_2_mask_[Clothes_custom_casual_M]                     = "masks/CG_casual_M/CG_casual_M_mask.jpg";
    map_clothestype_2_mask_[Clothes_custom_formal_M]                     = "masks/CG_formal_M/CG_formal_M_mask.jpg";
    map_clothestype_2_mask_[Clothes_custom_formal_W]                     = "masks/CG_formal_W/CG_formal_W_mask.jpg";
    map_clothestype_2_mask_[Clothes_custom_sport_M]                      = "masks/CG_sport_M/CG_sport_M_mask.jpg";
    map_clothestype_2_mask_[Clothes_custom_kitchen_M]                    = "masks/CG_kitchen_M/CG_kitchen_M_mask.jpg";
    map_clothestype_2_mask_[Clothes_self_jascha_casual_greensuit]        = "00_self/jascha_casual_greensuit/jascha_casual_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_jascha_longsleeve_greensuit]    = "00_self/jascha_longsleeve_greensuit/jascha_longsleeve_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_jascha_tshirt_greensuit]        = "00_self/jascha_tshirt_greensuit/jascha_tshirt_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_jascha_jeansandshoes_greensuit] = "00_self/jascha_jeansandshoes_greensuit/jascha_jeansandshoes_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_teresa_casual_greensuit]        = "00_self/teresa_casual_greensuit/teresa_casual_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_teresa_jeans_greensuit]         = "00_self/teresa_jeans_greensuit/teresa_jeans_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_teresa_top_greensuit]           = "00_self/teresa_top_greensuit/teresa_top_greensuit_mask.png";
    map_clothestype_2_mask_[Clothes_self_christina_casual_manual]        = "00_self/christina_casual_manual/christina_casual_manual_mask.png";
    map_clothestype_2_mask_[Clothes_self_andreas_casual_manual]          = "00_self/andreas_casual_manual/andreas_casual_manual_mask.png";
    map_clothestype_2_mask_[Clothes_self_matthias_casual_manual]         = "00_self/matthias_casual_manual/matthias_casual_manual_mask.png";


    // mapping clothes type to referencemodel
    map_clothestype_2_referencemodel_[Clothes_custom_casual_M]                     = "/reference_models/CG_M_ReferenceModel/CG_M_ReferenceModel.bim";
    map_clothestype_2_referencemodel_[Clothes_custom_formal_M]                     = "/reference_models/CG_M_ReferenceModel/CG_M_ReferenceModel.bim";
    map_clothestype_2_referencemodel_[Clothes_custom_formal_W]                     = "/reference_models/CG_M_ReferenceModel/CG_M_ReferenceModel.bim";
    map_clothestype_2_referencemodel_[Clothes_custom_sport_M]                      = "/reference_models/CG_M_ReferenceModel/CG_M_ReferenceModel.bim";
    map_clothestype_2_referencemodel_[Clothes_custom_kitchen_M]                    = "/reference_models/CG_M_ReferenceModel/CG_M_ReferenceModel.bim";
    map_clothestype_2_referencemodel_[Clothes_self_jascha_casual_greensuit]        = "/clothes/00_self/jascha_nude/jascha_nude.bim";
    map_clothestype_2_referencemodel_[Clothes_self_jascha_longsleeve_greensuit]    = "/clothes/00_self/jascha_nude/jascha_nude.bim";
    map_clothestype_2_referencemodel_[Clothes_self_jascha_tshirt_greensuit]        = "/clothes/00_self/jascha_nude/jascha_nude.bim";
    map_clothestype_2_referencemodel_[Clothes_self_jascha_jeansandshoes_greensuit] = "/clothes/00_self/jascha_nude/jascha_nude.bim";    
    map_clothestype_2_referencemodel_[Clothes_self_teresa_casual_greensuit]        = "/clothes/00_self/teresa_tight_clothes/teresa_tight_clothes.bim";
    map_clothestype_2_referencemodel_[Clothes_self_teresa_jeans_greensuit]         = "/clothes/00_self/teresa_tight_clothes/teresa_tight_clothes.bim";
    map_clothestype_2_referencemodel_[Clothes_self_teresa_top_greensuit]           = "/clothes/00_self/teresa_tight_clothes/teresa_tight_clothes.bim";
    map_clothestype_2_referencemodel_[Clothes_self_christina_casual_manual]        = "/clothes/00_self/christina_tight_clothes/christina_tight_clothes.bim";
    map_clothestype_2_referencemodel_[Clothes_self_andreas_casual_manual]          = "/clothes/00_self/andreas_nude/andreas_nude.bim";
    map_clothestype_2_referencemodel_[Clothes_self_matthias_casual_manual]         = "/clothes/00_self/matthias_nude/matthias_nude.bim";


    // mapping clothes type to fix-for-deformationtransfer
    map_clothestype_2_fixdeftrans_[Clothes_custom_casual_M]                     = "/selections/fix_deftrans_addclothes.sel";
    map_clothestype_2_fixdeftrans_[Clothes_custom_formal_M]                     = "/selections/fix_deftrans_addclothes.sel";
    map_clothestype_2_fixdeftrans_[Clothes_custom_formal_W]                     = "/selections/fix_deftrans_addclothes.sel";
    map_clothestype_2_fixdeftrans_[Clothes_custom_sport_M]                      = "/selections/fix_deftrans_addclothes.sel";
    map_clothestype_2_fixdeftrans_[Clothes_custom_kitchen_M]                    = "/selections/fix_deftrans_addclothes.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_jascha_casual_greensuit]        = "/clothes/00_self/jascha_casual_greensuit/jascha_casual_greensuit_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_jascha_longsleeve_greensuit]    = "/clothes/00_self/jascha_longsleeve_greensuit/jascha_longsleeve_greensuit_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_jascha_tshirt_greensuit]        = "/clothes/00_self/jascha_tshirt_greensuit/jascha_tshirt_greensuit_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_jascha_jeansandshoes_greensuit] = "/clothes/00_self/jascha_jeansandshoes_greensuit/jascha_jeansandshoes_greensuit_selection.sel"; 
    map_clothestype_2_fixdeftrans_[Clothes_self_teresa_casual_greensuit]        = "/clothes/00_self/teresa_casual_greensuit/teresa_casual_greensuit_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_teresa_jeans_greensuit]         = "/clothes/00_self/teresa_jeans_greensuit/teresa_jeans_greensuit_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_teresa_top_greensuit]           = "/clothes/00_self/teresa_top_greensuit/teresa_top_greensuit_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_christina_casual_manual]        = "/clothes/00_self/christina_casual_manual/christina_casual_manual_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_andreas_casual_manual]          = "/clothes/00_self/andreas_casual_manual/andreas_casual_manual_selection.sel";
    map_clothestype_2_fixdeftrans_[Clothes_self_matthias_casual_manual]         = "/clothes/00_self/matthias_casual_manual/matthias_casual_manual_selection.sel";


    // mapping from clothes type to self-scanned or not
    map_clothestype_2_selfscanned_[Clothes_custom_casual_M]                     = false;
    map_clothestype_2_selfscanned_[Clothes_custom_formal_M]                     = false;
    map_clothestype_2_selfscanned_[Clothes_custom_formal_W]                     = false;
    map_clothestype_2_selfscanned_[Clothes_custom_sport_M]                      = false;
    map_clothestype_2_selfscanned_[Clothes_custom_kitchen_M]                    = false;
    map_clothestype_2_selfscanned_[Clothes_self_jascha_casual_greensuit]        = true;
    map_clothestype_2_selfscanned_[Clothes_self_jascha_longsleeve_greensuit]    = true;
    map_clothestype_2_selfscanned_[Clothes_self_jascha_tshirt_greensuit]        = true;
    map_clothestype_2_selfscanned_[Clothes_self_jascha_jeansandshoes_greensuit] = true;
    map_clothestype_2_selfscanned_[Clothes_self_teresa_casual_greensuit]        = true;
    map_clothestype_2_selfscanned_[Clothes_self_teresa_jeans_greensuit]         = true;
    map_clothestype_2_selfscanned_[Clothes_self_teresa_top_greensuit]           = true;
    map_clothestype_2_selfscanned_[Clothes_self_christina_casual_manual]        = true;
    map_clothestype_2_selfscanned_[Clothes_self_andreas_casual_manual]          = true;
    map_clothestype_2_selfscanned_[Clothes_self_matthias_casual_manual]         = true;


    // mapping hairstyle type to filename
    map_hairstyletype_2_filename_[Hairstyle_M_02] = "CG_Hair_M_02_blond/CG_Hair_M_02_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_M_03] = "CG_Hair_M_03_blond/CG_Hair_M_03_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_M_07] = "CG_Hair_M_07_blond/CG_Hair_M_07_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_M_09] = "CG_Hair_M_09_blond/CG_Hair_M_09_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_M_17] = "CG_Hair_M_17_blond/CG_Hair_M_17_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_02] = "CG_Hair_W_02_blond/CG_Hair_W_02_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_05] = "CG_Hair_W_05_blond/CG_Hair_W_05_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_07] = "CG_Hair_W_07_blond/CG_Hair_W_07_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_10] = "CG_Hair_W_10_blond/CG_Hair_W_10_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_12] = "CG_Hair_W_12_blond/CG_Hair_W_12_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_19] = "CG_Hair_W_19_blond/CG_Hair_W_19_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_24] = "CG_Hair_W_24_blond/CG_Hair_W_24_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_28] = "CG_Hair_W_28_blond/CG_Hair_W_28_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_33] = "CG_Hair_W_33_blond/CG_Hair_W_33_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_34] = "CG_Hair_W_34_blond/CG_Hair_W_34_blond.bim";
    map_hairstyletype_2_filename_[Hairstyle_W_35] = "CG_Hair_W_35_blond/CG_Hair_W_35_blond.bim";


    // mapping hairstyle type to mask
    map_hairstyletype_2_mask_[Hairstyle_M_02] = "CG_Hair_M_02_black/CG_Hair_M_02_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_M_03] = "CG_Hair_M_03_black/CG_Hair_M_03_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_M_07] = "CG_Hair_M_07_black/CG_Hair_M_07_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_M_09] = "CG_Hair_M_09_black/CG_Hair_M_09_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_M_17] = "CG_Hair_M_17_black/CG_Hair_M_17_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_02] = "CG_Hair_W_02_black/CG_Hair_W_02_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_05] = "CG_Hair_W_05_black/CG_Hair_W_05_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_07] = "CG_Hair_W_07_black/CG_Hair_W_07_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_10] = "CG_Hair_W_10_black/CG_Hair_W_10_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_12] = "CG_Hair_W_12_black/CG_Hair_W_12_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_19] = "CG_Hair_W_19_black/CG_Hair_W_19_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_24] = "CG_Hair_W_24_black/CG_Hair_W_24_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_28] = "CG_Hair_W_28_black/CG_Hair_W_28_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_33] = "CG_Hair_W_33_black/CG_Hair_W_33_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_34] = "CG_Hair_W_34_black/CG_Hair_W_34_black_mask.jpg";
    map_hairstyletype_2_mask_[Hairstyle_W_35] = "CG_Hair_W_35_black/CG_Hair_W_35_black_mask.jpg";


    // mapping hairstyle type to wig
    map_hairstyletype_2_wig_[Hairstyle_M_02] = false;
    map_hairstyletype_2_wig_[Hairstyle_M_03] = false;
    map_hairstyletype_2_wig_[Hairstyle_M_07] = false;
    map_hairstyletype_2_wig_[Hairstyle_M_09] = false;
    map_hairstyletype_2_wig_[Hairstyle_M_17] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_02] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_05] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_07] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_10] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_12] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_19] = false;
    map_hairstyletype_2_wig_[Hairstyle_W_24] = true;
    map_hairstyletype_2_wig_[Hairstyle_W_28] = true;
    map_hairstyletype_2_wig_[Hairstyle_W_33] = true;
    map_hairstyletype_2_wig_[Hairstyle_W_34] = true;
    map_hairstyletype_2_wig_[Hairstyle_W_35] = true;


    // mapping hairstyle type and haircolor type to filename
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_02][Haircolor_blond] = "CG_Hair_M_02_blond/CG_Hair_M_02_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_02][Haircolor_brown] = "CG_Hair_M_02_brown/CG_Hair_M_02_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_02][Haircolor_gray]  = "CG_Hair_M_02_gray/CG_Hair_M_02_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_03][Haircolor_blond] = "CG_Hair_M_03_blond/CG_Hair_M_03_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_03][Haircolor_brown] = "CG_Hair_M_03_brown/CG_Hair_M_03_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_03][Haircolor_gray]  = "CG_Hair_M_03_gray/CG_Hair_M_03_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_07][Haircolor_blond] = "CG_Hair_M_07_blond/CG_Hair_M_07_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_07][Haircolor_brown] = "CG_Hair_M_07_brown/CG_Hair_M_07_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_07][Haircolor_gray]  = "CG_Hair_M_07_gray/CG_Hair_M_07_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_09][Haircolor_blond] = "CG_Hair_M_09_blond/CG_Hair_M_09_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_09][Haircolor_brown] = "CG_Hair_M_09_brown/CG_Hair_M_09_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_09][Haircolor_gray]  = "CG_Hair_M_09_gray/CG_Hair_M_09_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_17][Haircolor_blond] = "CG_Hair_M_17_blond/CG_Hair_M_17_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_17][Haircolor_brown] = "CG_Hair_M_17_lightbrown/CG_Hair_M_17_lightbrown_color.jpg"; // attention: use lightbrown instead or brown";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_M_17][Haircolor_gray]  = "CG_Hair_M_17_gray/CG_Hair_M_17_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_02][Haircolor_blond] = "CG_Hair_W_02_blond/CG_Hair_W_02_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_02][Haircolor_brown] = "CG_Hair_W_02_brown/CG_Hair_W_02_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_02][Haircolor_gray]  = "CG_Hair_W_02_gray/CG_Hair_W_02_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_05][Haircolor_blond] = "CG_Hair_W_05_blond/CG_Hair_W_05_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_05][Haircolor_brown] = "CG_Hair_W_05_lightbrown/CG_Hair_W_05_lightbrown_color.jpg"; // attention: use lightbrown instead or brown
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_05][Haircolor_gray]  = "CG_Hair_W_05_gray/CG_Hair_W_05_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_07][Haircolor_blond] = "CG_Hair_W_07_blond/CG_Hair_W_07_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_07][Haircolor_brown] = "CG_Hair_W_07_brown/CG_Hair_W_07_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_07][Haircolor_gray]  = "CG_Hair_W_07_gray/CG_Hair_W_07_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_10][Haircolor_blond] = "CG_Hair_W_10_blond/CG_Hair_W_10_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_10][Haircolor_brown] = "CG_Hair_W_10_brown/CG_Hair_W_10_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_10][Haircolor_gray]  = "CG_Hair_W_10_gray/CG_Hair_W_10_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_12][Haircolor_blond] = "CG_Hair_W_12_blond/CG_Hair_W_12_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_12][Haircolor_brown] = "CG_Hair_W_12_brown/CG_Hair_W_12_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_12][Haircolor_gray]  = "CG_Hair_W_12_gray/CG_Hair_W_12_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_19][Haircolor_blond] = "CG_Hair_W_19_blond/CG_Hair_W_19_blond_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_19][Haircolor_brown] = "CG_Hair_W_19_brown/CG_Hair_W_19_brown_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_19][Haircolor_gray]  = "CG_Hair_W_19_gray/CG_Hair_W_19_gray_color.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_24][Haircolor_blond] = "Haircolor_blond.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_24][Haircolor_brown] = "Haircolor_brown.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_24][Haircolor_gray]  = "Haircolor_gray.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_28][Haircolor_blond] = "Haircolor_blond.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_28][Haircolor_brown] = "Haircolor_brown.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_28][Haircolor_gray]  = "Haircolor_gray.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_33][Haircolor_blond] = "Haircolor_blond.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_33][Haircolor_brown] = "Haircolor_brown.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_33][Haircolor_gray]  = "Haircolor_gray.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_34][Haircolor_blond] = "Haircolor_blond.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_34][Haircolor_brown] = "Haircolor_brown.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_34][Haircolor_gray]  = "Haircolor_gray.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_35][Haircolor_blond] = "Haircolor_blond.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_35][Haircolor_brown] = "Haircolor_brown.jpg";
    map_hairstyle_and_colortype_2_filename_[Hairstyle_W_35][Haircolor_gray]  = "Haircolor_gray.jpg";
}


//-----------------------------------------------------------------------------


bool
Character_generator::
set_dir_database(const std::string dir_database)
{
    if (dir_database.empty())
    {
        return false;
    }
    else
    {
        dir_database_ = dir_database;

        return true;
    }
}


//-----------------------------------------------------------------------------


bool
Character_generator::
set_character_node(Character_node* character_node)
{
    if (character_node)
    {
        character_node_ = character_node;
        
        return true;
    }
    else
    {
        return false;
    }
}


//-----------------------------------------------------------------------------


std::string
Character_generator::
get_model_filename(const Model_type model_type)
{
    if (Model_keep != model_type && Model_backup != model_type)
    {
        std::string filename_model = dir_database_; filename_model.append("/CG_models/"); filename_model.append(map_modeltype_2_filename_[model_type]);

        return filename_model;
    }
    else
    {
        return "";
    }
}


//-----------------------------------------------------------------------------


bool
Character_generator::
set_skin(const Skin_type skin_type)
{
    std::cerr << "[DEBUG] in 'Character_generator::set_skin(...)' - A" << std::endl;

    if (!character_node_)
    {
        std::cerr << "[ERROR] Can't set skin. No character node set." << std::endl;
        return false;
    }

    std::string filename_skin_base = dir_database_; filename_skin_base.append("/CG_skins/");
                                     filename_skin_base.append(map_skintype_2_filename_[skin_type]);
    std::string filename_skin      = filename_skin_base; filename_skin.append("_color.jpg");
    std::string filename_skin_nm   = filename_skin_base; filename_skin_nm.append("_high_nm.jpg");
    std::string filename_skin_spec = filename_skin_base; filename_skin_spec.append("_spec.jpg");

    std::vector< graphene::gl::Texture* > textures_current_node = character_node_->get_textures();

    // modify all textures (exept wig)
    for (unsigned int i = 0; i < textures_current_node.size(); ++i)
    {
        cv::Mat texture_opencv = texture_to_opencv(textures_current_node[i]);

        graphene::utility::Texture_processing texture_processing;
        bool resized = false;

        if (textures_current_node[i]->name_.find("color.jpg") != std::string::npos)
        {
            if (textures_current_node[i]->name_.find("_wig_color.jpg") != std::string::npos)
            {
                continue;
            }

            // change skin
            cv::Mat img_skin_model = cv::imread(filename_skin.c_str());
            if(! img_skin_model.data )
            {
                std::cerr << "[ERROR] Can't read albedo texture (skin)!" << std::endl;
                return false;
            }

            // copy texture
            texture_processing.copy_and_paste(texture_opencv, img_skin_model, resized);
        }
        else if (textures_current_node[i]->name_.find("nm.jpg") != std::string::npos)
        {
            if (textures_current_node[i]->name_.find("_wig_nm.jpg") != std::string::npos)
            {
                continue;
            }

            // change skin
            cv::Mat img_skin_model = cv::imread(filename_skin_nm.c_str());
            if(! img_skin_model.data )
            {
                std::cerr << "[ERROR] Can't read normal texture (skin)!" << std::endl;
                return false;
            }

            // copy texture
            texture_processing.copy_and_paste(texture_opencv, img_skin_model, resized);
        }
        else if (textures_current_node[i]->name_.find("spec.jpg") != std::string::npos)
        {
            if (textures_current_node[i]->name_.find("_wig_spec.jpg") != std::string::npos)
            {
                continue;
            }

            // change skin
            cv::Mat img_skin_model = cv::imread(filename_skin_spec.c_str());
            if(! img_skin_model.data )
            {
                std::cerr << "[ERROR] Can't read specular texture (skin)!" << std::endl;
                return false;
            }

            // copy texture
            texture_processing.copy_and_paste(texture_opencv, img_skin_model, resized);
        }

        if (resized)
        {
            const unsigned int new_w = texture_opencv.cols;
            const unsigned int new_h = texture_opencv.rows;
            textures_current_node[i]->width_  = new_w;
            textures_current_node[i]->height_ = new_h;

            textures_current_node[i]->data_.resize(new_w * new_h * 4);
        }

        opencv_to_texture(texture_opencv, textures_current_node[i]);        
    }

    character_node_->update_textures();

    return true;
}


//-----------------------------------------------------------------------------


bool
Character_generator::
set_eyecolor(const Eyecolor_type eyecolor_type)
{
    std::cerr << "[DEBUG] in 'Character_generator::set_eyecolor(...)' - A" << std::endl;

    if (!character_node_)
    {
        std::cerr << "[ERROR] Can't set eye color. No character node set." << std::endl;
        return false;
    }

    std::string filename_eyecolor = dir_database_; filename_eyecolor.append("/CG_eyecolors/"); filename_eyecolor.append(map_eyecolortype_2_filename_[eyecolor_type]);

    // load mask
    std::string filename_mask = dir_database_; filename_mask.append("/CG_eyecolors/"); filename_mask.append("masks/CG_EyeColor_mask.jpg");
    cv::Mat mask_eyecolor = cv::imread(filename_mask.c_str(), 0);
    if(! mask_eyecolor.data )
    {
        std::cerr << "[ERROR] Can't read mask!: " << filename_mask << std::endl;
        return false;
    }

    std::vector< graphene::gl::Texture* > textures_current_node = character_node_->get_textures();

    for (unsigned int i = 0; i < textures_current_node.size(); ++i)
    {
        // modify albedo map
        if (textures_current_node[i]->name_.find("color.jpg") != std::string::npos)
        {
            if (textures_current_node[i]->name_.find("_wig_color.jpg") != std::string::npos)
            {
                continue;
            }

            cv::Mat texture_opencv = texture_to_opencv(textures_current_node[i]);

            // change eye color
            graphene::utility::Texture_processing texture_processing;
            cv::Mat img_eyecolor_model = cv::imread(filename_eyecolor.c_str());
            if(! img_eyecolor_model.data )
            {
                std::cerr << "[ERROR] Can't read albedo texture (eye color)!" << std::endl;
                return false;
            }

            // copy texture
            bool resized = false;
            texture_processing.copy_and_paste(texture_opencv, img_eyecolor_model, mask_eyecolor, resized);

            if (resized)
            {
                const unsigned int new_w = texture_opencv.cols;
                const unsigned int new_h = texture_opencv.rows;
                textures_current_node[i]->width_  = new_w;
                textures_current_node[i]->height_ = new_h;

                textures_current_node[i]->data_.resize(new_w * new_h * 4);
            }

            opencv_to_texture(texture_opencv, textures_current_node[i]);

            character_node_->update_textures();
            break;
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Character_generator::
add_clothes(const Clothes_type clothes_type, gl::GL_state* gls, const double chef_hat_scale, const double hair_offset, const double hair_offset2, const std::string filename_selection_subdivision)
{
    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - A" << std::endl;

    if (!character_node_)
    {
        std::cerr << "[ERROR] Can't add clothes. No character node set." << std::endl;
        return false;
    }

    std::string filename_clothes = dir_database_; filename_clothes.append("/clothes/"); filename_clothes.append(map_clothestype_2_filename_[clothes_type]);
    std::string filename_mask = dir_database_; filename_mask.append("/clothes/"); filename_mask.append(map_clothestype_2_mask_[clothes_type]);
    std::string filename_reference_model = dir_database_; filename_reference_model.append(map_clothestype_2_referencemodel_[clothes_type]);
    std::string filename_fix_deftrans_addclothes = dir_database_; filename_fix_deftrans_addclothes.append(map_clothestype_2_fixdeftrans_[clothes_type]);
    const bool  mode_selfscanned = map_clothestype_2_selfscanned_[clothes_type];

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - B" << std::endl;

    std::vector<size_t> vertices_fix_deftrans_addclothes;
    const bool ok_fix_deftrans_addclothes = graphene::surface_mesh::load_selection_from_file(vertices_fix_deftrans_addclothes, filename_fix_deftrans_addclothes);

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - C" << std::endl;

    // load clothes model mesh
    graphene::character::Character clothes_model;
    if (!clothes_model.read(filename_clothes.c_str()))
    {
        std::cerr << "[ERROR] Can't read clothes-model!" << std::endl;
        return false;
    }
    graphene::surface_mesh::Surface_mesh clothes_model_mesh = character_to_surface_mesh(clothes_model);
    if (!filename_selection_subdivision.empty())
    {
        std::cerr << "[DEBUG] in add clothes: before subdivision ; clothes_model_mesh.n_vertices(): " << clothes_model_mesh.n_vertices() << std::endl;
        load_selection_from_file(clothes_model_mesh, filename_selection_subdivision);
        graphene::surface_mesh::loop_subdivision(clothes_model_mesh);
        std::cerr << "[DEBUG] in add clothes: after subdivision ; clothes_model_mesh.n_vertices(): " << clothes_model_mesh.n_vertices() << std::endl;
    }


    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - D" << std::endl;


    // transfer texture(s)

    // load mask
    cv::Mat mask_clothes_model = cv::imread(filename_mask.c_str(), 0);
    if(! mask_clothes_model.data )
    {
        std::cerr << "[ERROR] Can't read mask!: " << filename_mask << std::endl;
        return false;
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - E" << std::endl;

    std::vector< graphene::gl::Texture* > textures_current_node = character_node_->get_textures();

    // modify all textures (exept wig)
    for (unsigned int i = 0; i < textures_current_node.size(); ++i)
    {
        cv::Mat texture_opencv = texture_to_opencv(textures_current_node[i]);

        graphene::utility::Texture_processing texture_processing;
        bool resized = false;

        if (textures_current_node[i]->name_.find("color.jpg") != std::string::npos  )
        {
            if (textures_current_node[i]->name_.find("_wig_color.jpg") != std::string::npos)
            {
                continue;
            }

            std::string texname_clothes_model = clothes_model.skins()[0]->get_mesh_property<std::string>("m:texturename")[0];

            cv::Mat img_clothes_model = cv::imread(texname_clothes_model.c_str());
            if(! img_clothes_model.data )
            {
                std::cerr << "[ERROR] Can't read albedo texture!" << std::endl;
                return false;
            }

            // copy texture
            texture_processing.copy_and_paste(texture_opencv, img_clothes_model, mask_clothes_model, resized);
        }
        else if (textures_current_node[i]->name_.find("nm.jpg") != std::string::npos  )
        {
            if (textures_current_node[i]->name_.find("_wig_nm.jpg") != std::string::npos)
            {
                continue;
            }

            std::string texname_clothes_model = clothes_model.skins()[0]->get_mesh_property<std::string>("m:texturename_nm")[0];

            cv::Mat img_clothes_model = cv::imread(texname_clothes_model.c_str());
            if(! img_clothes_model.data )
            {
                std::cerr << "[ERROR] Can't read normal map texture!" << std::endl;
                return false;
            }

            // copy texture
            texture_processing.copy_and_paste(texture_opencv, img_clothes_model, mask_clothes_model, resized);
        }
        else if (textures_current_node[i]->name_.find("spec.jpg") != std::string::npos  )
        {
            if (textures_current_node[i]->name_.find("_wig_spec.jpg") != std::string::npos)
            {
                continue;
            }

            std::string texname_clothes_model = clothes_model.skins()[0]->get_mesh_property<std::string>("m:texturename_spec")[0];

            cv::Mat img_clothes_model = cv::imread(texname_clothes_model.c_str());
            if(! img_clothes_model.data )
            {
                std::cerr << "[ERROR] Can't read specular map texture!" << std::endl;
                return false;
            }

            // copy texture
            texture_processing.copy_and_paste(texture_opencv, img_clothes_model, mask_clothes_model, resized);
        }
        else
        {
            std::cerr << "[ERROR] Should not happen! textures_current_node[i]->name_: " << textures_current_node[i]->name_ << std::endl;
            return false;
        }

        if (resized)
        {
            const unsigned int new_w = texture_opencv.cols;
            const unsigned int new_h = texture_opencv.rows;
            textures_current_node[i]->width_  = new_w;
            textures_current_node[i]->height_ = new_h;

            textures_current_node[i]->data_.resize(new_w * new_h * 4);
        }

        opencv_to_texture(texture_opencv, textures_current_node[i]);
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - F" << std::endl;

    character_node_->update_textures();

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - G" << std::endl;

    // transfer geometry

    // load reference surface_mesh
    graphene::character::Character reference_model;
    if (!reference_model.read(filename_reference_model.c_str()))
    {
        std::cerr << "[ERROR] Can't read reference model!" << std::endl;
        return false;
    }
    graphene::surface_mesh::Surface_mesh reference_model_mesh = character_to_surface_mesh(reference_model);
    if (!filename_selection_subdivision.empty())
    {
        load_selection_from_file(reference_model_mesh, filename_selection_subdivision);
        graphene::surface_mesh::loop_subdivision(reference_model_mesh);
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - H" << std::endl;

    // register clothes model to reference model
    {
        if (mode_selfscanned)
        {
            // compute optimal scale based on height of model (eyes and feet joints)
            graphene::character::Skeleton& clothes_model_skeleton   = clothes_model.skeleton();
            graphene::character::Skeleton& reference_model_skeleton = reference_model.skeleton();
            clothes_model_skeleton.init();
            reference_model_skeleton.init();
            const std::vector<graphene::character::Joint*>& clothes_model_joints = clothes_model_skeleton.joints_;
            const std::vector<graphene::character::Joint*>& reference_model_joints = reference_model_skeleton.joints_;
            std::vector< Point > clothes_model_points;
            std::vector< Point > reference_model_points;
            if (clothes_model_joints.size() != reference_model_joints.size())
            {
                std::cerr << "clothes_model_joints.size(): " << clothes_model_joints.size() << std::endl;
                std::cerr << "reference_model_joints.size(): " << reference_model_joints.size() << std::endl;
                exit(1);
            }
            for (unsigned int j_idx = 0; j_idx < clothes_model_joints.size(); ++j_idx)
            {
                if ( clothes_model_joints[j_idx]->get_name() == "l_eyeball_joint" ||
                     clothes_model_joints[j_idx]->get_name() == "r_eyeball_joint" ||
                     clothes_model_joints[j_idx]->get_name() == "l_ankle"         ||
                     clothes_model_joints[j_idx]->get_name() == "r_ankle"         ||
                     clothes_model_joints[j_idx]->get_name() == "l_metatarsal"    ||
                     clothes_model_joints[j_idx]->get_name() == "r_metatarsal" )
                {
//                    std::cerr << clothes_model_joints[j_idx]->get_name() << std::endl;
                    clothes_model_points.push_back( clothes_model_joints[j_idx]->get_global_translation() );
                    std::cerr << "clothes_model_joints[j_idx]->get_global_translation(): " << clothes_model_joints[j_idx]->get_global_translation() << std::endl;
                    reference_model_points.push_back( reference_model_joints[j_idx]->get_global_translation() );
                    std::cerr << "reference_model_joints[j_idx]->get_global_translation(): " << reference_model_joints[j_idx]->get_global_translation() << std::endl;
                }
            }

            Mat4f M_head_clothesmodel2referencemodel = graphene::geometry::registration(clothes_model_points, reference_model_points, graphene::geometry::CONFORMAL_REGISTRATION);
            transform_mesh(clothes_model_mesh, M_head_clothesmodel2referencemodel);

            // compute optimal rotation and translation based on all vertices
            clothes_model_points.clear();
            reference_model_points.clear();
            for (auto v : clothes_model_mesh.vertices())
            {
                clothes_model_points.push_back( clothes_model_mesh.position(v) );
                reference_model_points.push_back( reference_model_mesh.position(v) );
            }

            M_head_clothesmodel2referencemodel = graphene::geometry::registration(clothes_model_points, reference_model_points, graphene::geometry::RIGID_REGISTRATION);
            transform_mesh(clothes_model_mesh, M_head_clothesmodel2referencemodel);

            // non-rigidly deform clothes-model to reference-model
            /* TEST
            resolve_collisions(clothes_model_mesh,
                               reference_model_mesh,
                               vertices_fix_deftrans_addclothes);
            */

            // only for debugging: update clothes model with resolved collisions
            /*
            graphene::character::Blendshapes* clothes_model_body = clothes_model.blendshapes()[0];
            graphene::character::Surface_mesh_skin* clothes_model_body_skin = clothes_model_body->base();
            for (auto v : clothes_model_body_skin->vertices())
            {
                clothes_model_body_skin->position(v) = clothes_model_mesh.position(v);
            }
            clothes_model_body_skin->update_face_normals();
            clothes_model_body_skin->update_vertex_normals();
            std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - H1" << std::endl;
            clothes_model.write_bim("/home/jachenba/danach.bim");
            std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - H2" << std::endl;
            */
        }
        else
        {
            // register clothes model to reference model w.r.t. head selection
            std::vector<unsigned int> head_vec;
            std::string filename_selection_head = dir_database_; filename_selection_head.append("/selections/head-6185.sel");
            if (!graphene::surface_mesh::load_selection_from_file(head_vec, filename_selection_head))
            {
                std::cerr << "[DEBUG] Can't open selection for head." << std::endl;
                return false;
            }
            std::vector< Point > clothes_model_points;
            std::vector< Point > reference_model_points;
            for ( unsigned int i = 0; i < head_vec.size(); ++i)
            {
                graphene::surface_mesh::Surface_mesh::Vertex v(head_vec[i]);

                clothes_model_points.push_back( clothes_model_mesh.position(v) );
                reference_model_points.push_back( reference_model_mesh.position(v) );
            }

            Mat4f M_head_clothesmodel2referencemodel = graphene::geometry::registration(clothes_model_points, reference_model_points, graphene::geometry::CONFORMAL_REGISTRATION);
            transform_mesh(clothes_model_mesh, M_head_clothesmodel2referencemodel);
        }
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - I" << std::endl;

    graphene::character::Character& current_character = character_node_->character();
    graphene::character::Blendshapes& current_blendshapes_body = *current_character.blendshapes()[0];

    // modify base
    graphene::character::Surface_mesh_skin* base_mesh_skin = current_blendshapes_body.base();
    graphene::surface_mesh::Surface_mesh base_mesh = *base_mesh_skin;
    graphene::surface_mesh::Surface_mesh base_mesh_BAK = *base_mesh_skin; // TODO DO MORE EFFICIENT

    if (ok_fix_deftrans_addclothes)
    {
        Surface_mesh::Vertex_property<bool> selected = base_mesh.vertex_property<bool>("v:selected");
        for (auto v : base_mesh.vertices())
        {
            selected[v] = false;
        }
        for (unsigned int i = 0; i < vertices_fix_deftrans_addclothes.size(); ++i)
        {
            graphene::surface_mesh::Surface_mesh::Vertex v(vertices_fix_deftrans_addclothes[i]);
            selected[v] = true;
        }
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - J1" << std::endl;

    graphene::surface_mesh::Deformation_transfer_botsch deftrans;
    if (! deftrans.create_deformed_targets(reference_model_mesh,
                                           clothes_model_mesh,
                                           base_mesh))
    {
        std::cerr << "[DEBUG] in add clothes: reference_model_mesh.n_vertices(): " << reference_model_mesh.n_vertices() << std::endl;
        std::cerr << "[DEBUG] in add clothes: clothes_model_mesh.n_vertices(): "   << clothes_model_mesh.n_vertices() << std::endl;
        std::cerr << "[DEBUG] in add clothes: base_mesh.n_vertices(): "            << base_mesh.n_vertices() << std::endl;

        std::cerr << "[ERROR] Can't deform base mesh." << std::endl;
        return false;
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - J2" << std::endl;

    // resolve collisions for self-scanned clothing
    if (mode_selfscanned)
    {
        resolve_collisions(base_mesh,
                           base_mesh_BAK,
                           vertices_fix_deftrans_addclothes);
    }

    // update current character with new clothes
    for (auto v : base_mesh_skin->vertices())
    {
        base_mesh_skin->position(v) = base_mesh.position(v);
    }
    base_mesh_skin->update_face_normals();
    base_mesh_skin->update_vertex_normals();

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - K" << std::endl;

    // modify targets
    std::vector<graphene::character::Blendshapes::Target_shape*>& current_targets_body = current_blendshapes_body.targets();
    for (unsigned int bs = 0; bs < current_targets_body.size(); ++bs)
    {
        graphene::character::Blendshapes::Target_shape* tmp_mesh = current_targets_body[bs];
        graphene::surface_mesh::Surface_mesh target_mesh = *tmp_mesh;

        Surface_mesh::Vertex_property<bool> selected = target_mesh.vertex_property<bool>("v:selected", false);
        if (ok_fix_deftrans_addclothes)
        {
            for (unsigned int i = 0; i < vertices_fix_deftrans_addclothes.size(); ++i)
            {
                graphene::surface_mesh::Surface_mesh::Vertex v(vertices_fix_deftrans_addclothes[i]);
                selected[v] = true;
            }
        }

        if (mode_selfscanned)
        {
            for (auto v : target_mesh.vertices())
            {
                if (!selected[v])
                {
                    target_mesh.position(v) = base_mesh.position(v);
                }
            }
        }
        else
        {
            graphene::surface_mesh::Deformation_transfer_botsch deftrans;
            if (! deftrans.create_deformed_targets(reference_model_mesh,
                                                   clothes_model_mesh,
                                                   target_mesh))
            {
                std::cerr << "[ERROR] Can't deform target shape." << std::endl;
                return false;
            }
        }

        // update blendshape targets
        for (auto v : tmp_mesh->vertices())
        {
            tmp_mesh->position(v) = target_mesh.position(v);
        }
        tmp_mesh->update_face_normals();
        tmp_mesh->update_vertex_normals();
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - L" << std::endl;

    if (ok_fix_deftrans_addclothes)
    {
        Surface_mesh::Vertex_property<bool> selected = base_mesh.get_vertex_property<bool>("v:selected");
        for (unsigned int i = 0; i < vertices_fix_deftrans_addclothes.size(); ++i)
        {
            graphene::surface_mesh::Surface_mesh::Vertex v(vertices_fix_deftrans_addclothes[i]);
            selected[v] = false;
        }
    }


    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - M" << std::endl;


    // eventually add skin (chef hat)
    if (Clothes_custom_kitchen_M == clothes_type)
    {
        std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - N" << std::endl;

        const bool has_wig = map_hairstyletype_2_wig_[latest_hairstyle_type_];

        std::vector<graphene::character::Surface_mesh_skin*>& skins_current_character = current_character.skins();

        std::string filename_chef_hat_mesh = dir_database_; filename_chef_hat_mesh.append("/clothes/ChefHat/ChefHat.obj");
        graphene::character::Surface_mesh_skin* chef_hat_skin = new graphene::character::Surface_mesh_skin;
        chef_hat_skin->read(filename_chef_hat_mesh.c_str());

        // transform chef hat
        std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - O" << std::endl;
        {
            // similarity transform
            graphene::character::Blendshapes* blendshapes_current_model = current_character.blendshapes()[0]; // TODO TODO TODO s.o.
            graphene::surface_mesh::Surface_mesh base_current_model = *blendshapes_current_model->base();     // TODO TODO TODO s.o.

            auto base_current_model_points = base_current_model.get_vertex_property<Point>("v:point");

            std::vector< Point > chef_hat_points_4_simil;
            graphene::surface_mesh::Surface_mesh::Vertex v_src_1(54011);
            graphene::surface_mesh::Surface_mesh::Vertex v_src_2(54147);
            graphene::surface_mesh::Surface_mesh::Vertex v_src_3(53725);
            graphene::surface_mesh::Surface_mesh::Vertex v_src_4(53869);
            chef_hat_points_4_simil.push_back(chef_hat_skin->position(v_src_1));
            chef_hat_points_4_simil.push_back(chef_hat_skin->position(v_src_2));
            chef_hat_points_4_simil.push_back(chef_hat_skin->position(v_src_3));
            chef_hat_points_4_simil.push_back(chef_hat_skin->position(v_src_4));

            std::vector< Point > base_current_model_points_4_simil;
            graphene::surface_mesh::Surface_mesh::Vertex v_tar_1( 8049 );
            graphene::surface_mesh::Surface_mesh::Vertex v_tar_2( 6741 );
            graphene::surface_mesh::Surface_mesh::Vertex v_tar_3( 7986 );
            graphene::surface_mesh::Surface_mesh::Vertex v_tar_4( 17275 );

            base_current_model_points_4_simil.push_back(base_current_model.position(v_tar_1));
            base_current_model_points_4_simil.push_back(base_current_model.position(v_tar_2));
            base_current_model_points_4_simil.push_back(base_current_model.position(v_tar_3));
            base_current_model_points_4_simil.push_back(base_current_model.position(v_tar_4));

            Mat4f M_chef_hat = graphene::geometry::registration(chef_hat_points_4_simil, base_current_model_points_4_simil, graphene::geometry::R_TRANS_SCALE_ANISO_XZ);
            transform_mesh(*chef_hat_skin, M_chef_hat);

            std::vector<unsigned int> chef_hat_inner_idx_vec;
            std::string filename_selection_chef_hat_inner = dir_database_; filename_selection_chef_hat_inner.append("/clothes/ChefHat/selection_chef_hat_inner.sel");
            if (!graphene::surface_mesh::load_selection_from_file(chef_hat_inner_idx_vec, filename_selection_chef_hat_inner))
            {
                std::cerr << "[DEBUG] Can't open selection for inner chef hat. Filename: " << filename_selection_chef_hat_inner << std::endl;
                return false;
            }

            graphene::surface_mesh::Triangle_kD_tree base_current_model_kd(base_current_model);

            std::vector< Point > src_points;
            std::vector< Point > tar_points;
            for ( unsigned int i = 0; i < chef_hat_inner_idx_vec.size(); ++i)
            {
                graphene::surface_mesh::Surface_mesh::Vertex v(chef_hat_inner_idx_vec[i]);

                src_points.push_back( chef_hat_skin->position(v) );

                const auto nn_tri = base_current_model_kd.nearest( chef_hat_skin->position(v) );

                tar_points.push_back( nn_tri.nearest );
            }

            std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - P" << std::endl;
            //        graphene::surface_mesh::RBF_deformer rbf_deformer(*chef_hat_skin, src_points, tar_points);
            M_chef_hat = graphene::geometry::registration(src_points, tar_points, graphene::geometry::R_TRANS_SCALE_ANISO_XZ);

            // heuristic: if there is a wig, slightly extend size
            if (has_wig)
            {
                for (int j = 0; j < 3; ++j)
                {
                    M_chef_hat(0, j) *= chef_hat_scale;
                    M_chef_hat(2, j) *= chef_hat_scale;
                }
            }
            transform_mesh(*chef_hat_skin, M_chef_hat);
            chef_hat_skin->update_face_normals();
            chef_hat_skin->update_vertex_normals();

            auto v_depends = chef_hat_skin->vertex_property<Vec4f>("v:skin_depend");
            auto v_weights = chef_hat_skin->vertex_property<Vec4f>("v:skin_weight");
            for (auto v : chef_hat_skin->vertices())
            {
                v_depends[v] = Vec4f(8, 0, 0, 0);
                v_weights[v] = Vec4f(1, 0, 0, 0);
            }
            auto h_normals = chef_hat_skin->halfedge_property<Normal>("h:normal");
            auto v_normals = chef_hat_skin->get_vertex_property<Normal>("v:normal");
            if (!v_normals)
            {
                std::cerr << "NO VERTEX NORMALS" << std::endl;
                return false;
            }
            for (auto h : chef_hat_skin->halfedges())
            {
                auto v = chef_hat_skin->to_vertex(h);

                h_normals[h] = v_normals[v];
            }
        }

        // add textures to texture heap
        std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - Q" << std::endl;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname_nm;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname_spec;
        texname = chef_hat_skin->get_mesh_property<std::string>("m:texturename");
        if (texname)
        {
            const std::string& texfilename = texname[0];
            //if texture already loaded, it should be on the heap
            gl::Texture* texture = gls->get_texture_from_heap(texfilename);
            //if not, load it
            if (texture == nullptr)
            {
                texture = new gl::Texture;
                if (texture->read(texfilename))
                {
                    //and add to texture heap
                    gls->add_texture_to_heap(texture);
                }
                else
                {
                    delete texture;
                }
            }
        }
        texname_nm = chef_hat_skin->get_mesh_property<std::string>("m:texturename_nm");
        if (texname_nm)
        {
            const std::string& texfilename = texname_nm[0];
            //if texture already loaded, it should be on the heap
            gl::Texture* texture = gls->get_texture_from_heap(texfilename);
            //if not, load it
            if (texture == nullptr)
            {
                texture = new gl::Texture;
                if (texture->read(texfilename))
                {
                    //and add to texture heap
                    gls->add_texture_to_heap(texture);
                }
                else
                {
                    delete texture;
                }
            }
        }
        texname_spec = chef_hat_skin->get_mesh_property<std::string>("m:texturename_spec");
        if (texname_spec)
        {
            const std::string& texfilename = texname_spec[0];
            //if texture already loaded, it should be on the heap
            gl::Texture* texture = gls->get_texture_from_heap(texfilename);
            //if not, load it
            if (texture == nullptr)
            {
                texture = new gl::Texture;
                if (texture->read(texfilename))
                {
                    //and add to texture heap
                    gls->add_texture_to_heap(texture);
                }
                else
                {
                    delete texture;
                }
            }
        }

        auto chef_hat_id = chef_hat_skin->mesh_property<std::string>("m:id");
        chef_hat_id[0] = "ChefHat";
        chef_hat_skin->init_properties();
        skins_current_character.push_back(chef_hat_skin);

        // add skin to GL_state
        std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - R" << std::endl;
        character_node_->add_skin(chef_hat_skin, gls);


        // correct hair if there is a chef hat and project hair (or wig) under that hat
        const bool with_hair_correction = true;
        if (with_hair_correction)
        {
            if (Hairstyle_keep == latest_hairstyle_type_)
            {
                std::cout << "[WARNING] Can't correct hair under chef hat. Set hairstyle first!" << std::endl;
                return true;
            }

            std::vector<graphene::surface_mesh::Surface_mesh::Face> chef_hat_bottom_faces_vec;
            {
                std::vector<unsigned int> chef_hat_bottom_idx_vec;
                std::string filename_chef_hat_bottom = dir_database_; filename_chef_hat_bottom.append("/clothes/ChefHat/selection_chef_hat_bottom.sel");
                if (!graphene::surface_mesh::load_selection_from_file(chef_hat_bottom_idx_vec, filename_chef_hat_bottom))
                {
                    std::cerr << "[DEBUG] Can't open selection for chef hat bottom." << std::endl;
                    return false;
                }

                // std::vector<Point> chef_hat_bottom_points;
                for ( unsigned int i = 0; i < chef_hat_bottom_idx_vec.size(); ++i) // TODO DURCH FACES !!! SET ANSTATT VECTOR ?!
                {
                    graphene::surface_mesh::Surface_mesh::Vertex v(chef_hat_bottom_idx_vec[i]);

                    // chef_hat_bottom_points.push_back(chef_hat_skin->position(v));

                    for (auto f : chef_hat_skin->faces(v) )
                    {
                        chef_hat_bottom_faces_vec.push_back(f);
                    }
                }
            }


            std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - S" << std::endl;
            std::vector<graphene::surface_mesh::Surface_mesh::Face> chef_hat_inner_faces_vec;
            {
                std::vector<unsigned int> chef_hat_inside_area_idx_vec;
                std::string filename_chef_hat_inside_area = dir_database_; filename_chef_hat_inside_area.append("/clothes/ChefHat/selection_chef_hat_inside_area.sel");
                if (!graphene::surface_mesh::load_selection_from_file(chef_hat_inside_area_idx_vec, filename_chef_hat_inside_area))
                {
                    std::cerr << "[DEBUG] Can't open selection for chef hat inside area." << std::endl;
                    return false;
                }

                auto chef_hat_inside_vertices = chef_hat_skin->vertex_property<bool>("v:inside", false);
                for ( unsigned int i = 0; i < chef_hat_inside_area_idx_vec.size(); ++i)
                {
                    graphene::surface_mesh::Surface_mesh::Vertex v(chef_hat_inside_area_idx_vec[i]);
                    chef_hat_inside_vertices[v] = true;
                }
                for ( auto f : chef_hat_skin->faces() )
                {
                    auto fvit = chef_hat_skin->vertices(f);
                    const auto v0 = *fvit;
                    const auto v1 = *(++fvit);
                    const auto v2 = *(++fvit);

                    if (chef_hat_inside_vertices[v0] ||
                        chef_hat_inside_vertices[v1] ||
                        chef_hat_inside_vertices[v2])
                    {
                        chef_hat_inner_faces_vec.push_back(f);
                    }
                }
            }


            if (!has_wig)
            {
                std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - T" << std::endl;
                std::vector<unsigned int> upperhead_no_ears_idx_vec;
                std::string filename_upperhead_no_ears = dir_database_; filename_upperhead_no_ears.append("/selections_wip/selection_upperhead_no_ears/selection_upperhead_no_ears.sel");
                if (!graphene::surface_mesh::load_selection_from_file(upperhead_no_ears_idx_vec, filename_upperhead_no_ears))
                {
                    std::cerr << "[DEBUG] Can't open selection for upper head without ears." << std::endl;
                    return false;
                }

                for ( unsigned int i = 0; i < upperhead_no_ears_idx_vec.size(); ++i)
                {
                    graphene::surface_mesh::Surface_mesh::Vertex v(upperhead_no_ears_idx_vec[i]);

                    const Point hair_point = base_mesh_skin->position(v);

                    Point nearest_bottom_point(0,0,0);
                    double dist = DBL_MAX;
                    for (unsigned int j = 0; j < chef_hat_bottom_faces_vec.size(); ++j)
                    {
                        auto fvit = chef_hat_skin->vertices(chef_hat_bottom_faces_vec[j]);
                        const auto v0 = *fvit;
                        const auto v1 = *(++fvit);
                        const auto v2 = *(++fvit);
                        const Point p0 = chef_hat_skin->position(v0);
                        const Point p1 = chef_hat_skin->position(v1);
                        const Point p2 = chef_hat_skin->position(v2);
                        Point nearest;
                        double tmp_dist = graphene::geometry::dist_point_triangle(hair_point, p0, p1, p2, nearest);
                        if (tmp_dist < dist)
                        {
                            dist = tmp_dist;
                            nearest_bottom_point = nearest;
                        }
                    }

                    const double dist_hair_point_nearest_bottom = distance(hair_point, nearest_bottom_point);

                    const bool hair_point_is_above = (hair_point[1] > nearest_bottom_point[1]) || (hair_point[1] > nearest_bottom_point[1]) || (hair_point[1] > nearest_bottom_point[1]);
                    if ( hair_point_is_above)
                    {
                        // projection to inner faces
                        Point nearest_inner_point(0,0,0);
                        double dist = DBL_MAX;
                        for (unsigned int c = 0; c < chef_hat_inner_faces_vec.size(); ++c)
                        {
                            auto fvit = chef_hat_skin->vertices(chef_hat_inner_faces_vec[c]);
                            const auto v0 = *fvit;
                            const auto v1 = *(++fvit);
                            const auto v2 = *(++fvit);
                            const Point p0 = chef_hat_skin->position(v0);
                            const Point p1 = chef_hat_skin->position(v1);
                            const Point p2 = chef_hat_skin->position(v2);

                            Point nearest;
                            double tmp_dist = graphene::geometry::dist_point_triangle(hair_point, p0, p1, p2, nearest);
                            if (tmp_dist < dist)
                            {
                                dist = tmp_dist;
                                nearest_inner_point = nearest;
                            }
                        }

                        const double dist_hair_point_nearest_inner = distance(hair_point, nearest_inner_point);

                        Point hair_point_target(0,0,0);
                        if ( dist_hair_point_nearest_bottom < dist_hair_point_nearest_inner + 0.01 )
                        {
                            hair_point_target = nearest_bottom_point;
                        }
                        else
                        {
                            hair_point_target = nearest_inner_point;
                        }
                        base_mesh_skin->position(v) = hair_point_target;


                        std::vector<graphene::character::Blendshapes::Target_shape*>& current_targets_body = current_blendshapes_body.targets();
                        for (unsigned int bs = 0; bs < current_targets_body.size(); ++bs)
                        {
                            graphene::character::Blendshapes::Target_shape* tmp_mesh = current_targets_body[bs];
                            tmp_mesh->position(v) = hair_point_target;
                        }
                    }
                }


                // TODO Targets TODO !!! NORMALEN UND BASE MESH NORMALEN


            }
            else
            {
                std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - U" << std::endl;
                graphene::character::Surface_mesh_skin* new_wig = skins_current_character[skins_current_character.size()-2]; // TODO

                for ( auto v_wig : new_wig->vertices() )
                {
                    const Point hair_point = new_wig->position(v_wig);

                    Point nearest_bottom_point(0,0,0);
                    double dist = DBL_MAX;
                    for (unsigned int j = 0; j < chef_hat_bottom_faces_vec.size(); ++j)
                    {
                        auto fvit = chef_hat_skin->vertices(chef_hat_bottom_faces_vec[j]);
                        const auto v0 = *fvit;
                        const auto v1 = *(++fvit);
                        const auto v2 = *(++fvit);
                        const Point p0 = chef_hat_skin->position(v0);
                        const Point p1 = chef_hat_skin->position(v1);
                        const Point p2 = chef_hat_skin->position(v2);

                        Point nearest;
                        double tmp_dist = graphene::geometry::dist_point_triangle(hair_point, p0, p1, p2, nearest);
                        if (tmp_dist < dist)
                        {
                            dist = tmp_dist;
                            nearest_bottom_point = nearest;
                        }
                    }

                    const double dist_hair_point_nearest_bottom = distance(hair_point, nearest_bottom_point);


                    const bool hair_point_is_above = (hair_point[1] > nearest_bottom_point[1]) || (hair_point[1] > nearest_bottom_point[1]) || (hair_point[1] > nearest_bottom_point[1]);

                    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - V" << std::endl;

                    if ( hair_point_is_above)
                    {
                        // projection to inner faces
                        Point nearest_inner_point(0,0,0);
                        double dist = DBL_MAX;
                        for (unsigned int c = 0; c < chef_hat_inner_faces_vec.size(); ++c)
                        {
                            auto fvit = chef_hat_skin->vertices(chef_hat_inner_faces_vec[c]);
                            const auto v0 = *fvit;
                            const auto v1 = *(++fvit);
                            const auto v2 = *(++fvit);
                            const Point p0 = chef_hat_skin->position(v0);
                            const Point p1 = chef_hat_skin->position(v1);
                            const Point p2 = chef_hat_skin->position(v2);

                            Point nearest;
                            double tmp_dist = graphene::geometry::dist_point_triangle(hair_point, p0, p1, p2, nearest);
                            if (tmp_dist < dist)
                            {
                                dist = tmp_dist;
                                nearest_inner_point = nearest;
                            }
                        }

                        const double dist_hair_point_nearest_inner = distance(hair_point, nearest_inner_point);

                        Point hair_point_target(0,0,0);
                        //                    if ( dist_hair_point_nearest_bottom < dist_hair_point_nearest_inner + 0.02 ) // TODO evtl. muss 0.01 höher sein !!!
                        if ( dist_hair_point_nearest_bottom < dist_hair_point_nearest_inner + hair_offset )
                        {
                            hair_point_target = nearest_bottom_point;
                        }
                        else
                        {
                            hair_point_target = nearest_inner_point;
                        }
                        new_wig->position(v_wig) = hair_point_target;
                    }


                    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - W" << std::endl;

                    const bool hair_point_is_critical = !hair_point_is_above && (hair_point[1] >= (nearest_bottom_point[1] - hair_offset2)); // i.e., 0.02 -> 2mm
                    if ( hair_point_is_critical )
                    {
                        //                    const Vec3d direction = (nearest_bottom_point - hair_point).normalize();
                        const Vec3f direction = nearest_bottom_point - hair_point;

                        //                    dist_hair_point_nearest_bottom

                        const Point hair_point_target = hair_point + 0.8*direction;
                        //                    const Point hair_point_target = nearest_bottom_point;
                        std::cerr << "hair_point_target: " << hair_point_target << std::endl;
                        new_wig->position(v_wig) = hair_point_target;
                    }
                }


                // TODO Targets TODO !!!


            }
        }
    }

    std::cerr << "[DEBUG] in 'Character_generator::add_clothes(...)' - Z" << std::endl;

    return true;
}


//-----------------------------------------------------------------------------


bool
Character_generator::
add_hairstyle(const Hairstyle_type hairstyle_type, gl::GL_state* gls, const std::string filename_selection_subdivision)
{
    std::cerr << "[DEBUG] in 'Character_generator::add_hairstyle(...)' - A" << std::endl;

    if (!character_node_)
    {
        std::cerr << "[ERROR] Can't add hairstyle. No character node set." << std::endl;
        return false;
    }

    if (Hairstyle_keep == hairstyle_type)
    {
        std::cout << "[WARNING] You don't seem to intend to change the hairstyle." << std::endl;
        return true;
    }

    latest_hairstyle_type_ = hairstyle_type;

    // check if there is a wig
    const bool has_wig = map_hairstyletype_2_wig_[hairstyle_type];

    graphene::character::Surface_mesh_skin* new_wig = 0;

    std::string filename_hairstyle = dir_database_; filename_hairstyle.append("/hair/"); filename_hairstyle.append(map_hairstyletype_2_filename_[hairstyle_type]);
    std::string filename_mask = dir_database_; filename_mask.append("/hair/masks/"); filename_mask.append(map_hairstyletype_2_mask_[hairstyle_type]);


    // load hairstyle_model mesh // TODO NUR BEI WIG?
    graphene::character::Character hairstyle_model;
    if (!hairstyle_model.read(filename_hairstyle.c_str()))
    {
        return false;
    }
    graphene::surface_mesh::Surface_mesh hairstyle_model_mesh = character_to_surface_mesh(hairstyle_model);
    if (!filename_selection_subdivision.empty())
    {
        std::cerr << "[DEBUG] in add hairstyle: before subdivision ; hairstyle_model_mesh.n_vertices(): " << hairstyle_model_mesh.n_vertices() << std::endl;
        load_selection_from_file(hairstyle_model_mesh, filename_selection_subdivision);
        graphene::surface_mesh::loop_subdivision(hairstyle_model_mesh);
        std::cerr << "[DEBUG] in add hairstyle: after subdivision ; hairstyle_model_mesh.n_vertices(): " << hairstyle_model_mesh.n_vertices() << std::endl;
    }

    graphene::character::Character& current_character = character_node_->character();
    graphene::character::Blendshapes& current_blendshapes_body = *current_character.blendshapes()[0];


    if (!has_wig)
    {
        // transfer textures

        // load mask of hairstyle
        cv::Mat mask_hairstyle_model = cv::imread(filename_mask.c_str(), 0);
        if(! mask_hairstyle_model.data )
        {
            std::cerr << "[ERROR] Can't read mask (hairstyle)!: " << filename_mask << std::endl;
            return false;
        }

        std::vector< graphene::gl::Texture* > textures_current_node = character_node_->get_textures();

        // modify all textures (exept albedo map and wig)
        for (unsigned int i = 0; i < textures_current_node.size(); ++i)
        {
            cv::Mat texture_opencv = texture_to_opencv(textures_current_node[i]);

            graphene::utility::Texture_processing texture_processing;
            bool resized = false;

            if (textures_current_node[i]->name_.find("color.jpg") != std::string::npos)
            {
                if (textures_current_node[i]->name_.find("_wig_color.jpg") != std::string::npos)
                {
                    continue;
                }

                continue; // skip albedo (albedo is set in 'set_haircolor(...)')

                std::string texname_hairstyle_model = hairstyle_model.skins()[0]->get_mesh_property<std::string>("m:texturename")[0];

                cv::Mat img_hairstyle_model = cv::imread(texname_hairstyle_model.c_str());
                if(! img_hairstyle_model.data )
                {
                    std::cerr << "[ERROR] Can't read albedo texture (hairstyle)!" << std::endl;
                    return false;
                }

                // copy texture
                texture_processing.copy_and_paste(texture_opencv, img_hairstyle_model, mask_hairstyle_model, resized);
            }
            else if (textures_current_node[i]->name_.find("nm.jpg") != std::string::npos)
            {
                if (textures_current_node[i]->name_.find("_wig_nm.jpg") != std::string::npos)
                {
                    continue;
                }

                std::string texname_hairstyle_model = hairstyle_model.skins()[0]->get_mesh_property<std::string>("m:texturename_nm")[0];

                cv::Mat img_hairstyle_model = cv::imread(texname_hairstyle_model.c_str());
                if(! img_hairstyle_model.data )
                {
                    std::cerr << "[ERROR] Can't read normal map texture (hairstyle)!" << std::endl;
                    return false;
                }

                // copy texture
                texture_processing.copy_and_paste(texture_opencv, img_hairstyle_model, mask_hairstyle_model, resized);
            }
            else if (textures_current_node[i]->name_.find("spec.jpg") != std::string::npos)
            {
                if (textures_current_node[i]->name_.find("_wig_spec.jpg") != std::string::npos)
                {
                    continue;
                }

                std::string texname_hairstyle_model = hairstyle_model.skins()[0]->get_mesh_property<std::string>("m:texturename_spec")[0];

                cv::Mat img_hairstyle_model = cv::imread(texname_hairstyle_model.c_str());
                if(! img_hairstyle_model.data )
                {
                    std::cerr << "[ERROR] Can't read specular map texture (hairstyle)!" << std::endl;
                    return false;
                }

                // copy texture
                texture_processing.copy_and_paste(texture_opencv, img_hairstyle_model, mask_hairstyle_model, resized);
            }
            else
            {
                std::cerr << "[ERROR] Should not happen! textures_current_node[i]->name_: " << textures_current_node[i]->name_ << std::endl;
                return false;
            }

            if (resized)
            {
                const unsigned int new_w = texture_opencv.cols;
                const unsigned int new_h = texture_opencv.rows;
                textures_current_node[i]->width_  = new_w;
                textures_current_node[i]->height_ = new_h;

                textures_current_node[i]->data_.resize(new_w * new_h * 4);
            }

            opencv_to_texture(texture_opencv, textures_current_node[i]);
        }

        character_node_->update_textures();

        // transfer geometry

        // load reference mesh
        std::string filename_reference_model = dir_database_; filename_reference_model.append("/reference_models/CG_M_ReferenceModel/20160830_old/CG_M_ReferenceModel.bim");
        graphene::character::Character reference_model;
        if (!reference_model.read(filename_reference_model.c_str()))
        {
            std::cerr << "[ERROR] Can't read reference model!" << std::endl;
            return false;
        }
        graphene::surface_mesh::Surface_mesh reference_model_mesh = character_to_surface_mesh(reference_model);
        if (!filename_selection_subdivision.empty())
        {
            load_selection_from_file(reference_model_mesh, filename_selection_subdivision);
            graphene::surface_mesh::loop_subdivision(reference_model_mesh);
        }

        // register hairstyle model to reference model
        {
            // load selection of body
            std::vector<unsigned int> body_vec;
            std::string filename_selection_body = dir_database_; filename_selection_body.append("/selections/body-14367.sel");
            if (!graphene::surface_mesh::load_selection_from_file(body_vec, filename_selection_body))
            {
                std::cerr << "[DEBUG] Can't open selection for body." << std::endl;
                return false;
            }
            std::vector< Point > hairstyle_model_points;
            std::vector< Point > reference_model_points;
            for ( unsigned int i = 0; i < body_vec.size(); ++i)
            {
                graphene::surface_mesh::Surface_mesh::Vertex v(body_vec[i]);

                hairstyle_model_points.push_back( hairstyle_model_mesh.position(v) );
                reference_model_points.push_back( reference_model_mesh.position(v) );
            }
            // similarity transformation
            const Mat4f M_body_hairmodel2referencemodel = graphene::geometry::registration(hairstyle_model_points, reference_model_points, graphene::geometry::CONFORMAL_REGISTRATION);
            transform_mesh(hairstyle_model_mesh, M_body_hairmodel2referencemodel);
        }


        // modify base
        graphene::character::Surface_mesh_skin* base_mesh_skin = current_blendshapes_body.base(); // TODO NACH OBEN ?!
        graphene::surface_mesh::Surface_mesh base_mesh = *base_mesh_skin;

        graphene::surface_mesh::Deformation_transfer_botsch deftrans;
        if (! deftrans.create_deformed_targets(reference_model_mesh,
                                               hairstyle_model_mesh,
                                               base_mesh))
        {
            std::cerr << "[DEBUG] in add hairstyle: reference_model_mesh.n_vertices(): " << reference_model_mesh.n_vertices() << std::endl;
            std::cerr << "[DEBUG] in add hairstyle: hairstyle_model_mesh.n_vertices(): " << hairstyle_model_mesh.n_vertices() << std::endl;
            std::cerr << "[DEBUG] in add hairstyle: base_mesh.n_vertices(): "            << base_mesh.n_vertices() << std::endl;

            std::cerr << "[ERROR] Can't deform base mesh." << std::endl;
            return false;
        }

        for (auto v : base_mesh_skin->vertices())
        {
            base_mesh_skin->position(v) = base_mesh.position(v);
        }
        base_mesh_skin->update_face_normals();
        base_mesh_skin->update_vertex_normals();

        // modify targets
        std::vector<graphene::character::Blendshapes::Target_shape*>& current_targets_body = current_blendshapes_body.targets();

        for (unsigned int bs = 0; bs < current_targets_body.size(); ++bs)
        {
            graphene::character::Blendshapes::Target_shape* tmp_mesh = current_targets_body[bs];
            graphene::surface_mesh::Surface_mesh target_mesh = *tmp_mesh;

            graphene::surface_mesh::Deformation_transfer_botsch deftrans;
            if (! deftrans.create_deformed_targets(reference_model_mesh,
                                                   hairstyle_model_mesh,
                                                   target_mesh))
            {
                std::cerr << "[ERROR] Can't deform target shape." << std::endl;
                return false;
            }

            // update blendshape targets
            for (auto v : tmp_mesh->vertices())
            {
                tmp_mesh->position(v) = target_mesh.position(v);
            }
            tmp_mesh->update_face_normals();
            tmp_mesh->update_vertex_normals();
        }            
    }
    else
    {
        // copy wig from character 'hairstyle_model' to character node
        std::vector<graphene::character::Surface_mesh_skin*>& skins_hairstyle_model = hairstyle_model.skins();
        graphene::character::Surface_mesh_skin* wig_skin_hairstyle_model = skins_hairstyle_model.back();


        std::vector<graphene::character::Surface_mesh_skin*>& skins_current_character = current_character.skins();
        new_wig = new graphene::character::Surface_mesh_skin;
        *new_wig = *wig_skin_hairstyle_model;
        new_wig->init_properties();
        skins_current_character.push_back(new_wig); // TODO CHARACTER GUI NEU AUFBAUEN ?!?!

        // copy mesh properties for textures if available
        // TODO  better generate a new file name ?!?!
        auto prop_hairstyle_model_wig_albedo = wig_skin_hairstyle_model->get_mesh_property<std::string>("m:texturename");
        if (prop_hairstyle_model_wig_albedo)
        {
            auto prop_tex_albedo = new_wig->mesh_property<std::string>("m:texturename");
            prop_tex_albedo[0] = prop_hairstyle_model_wig_albedo[0];
        }
        auto prop_hairstyle_model_wig_nm = wig_skin_hairstyle_model->get_mesh_property<std::string>("m:texturename_nm");
        if (prop_hairstyle_model_wig_nm)
        {
            auto prop_tex_nm = new_wig->mesh_property<std::string>("m:texturename_nm");
            prop_tex_nm[0] = prop_hairstyle_model_wig_nm[0];
        }
        auto prop_hairstyle_model_wig_spec = wig_skin_hairstyle_model->get_mesh_property<std::string>("m:texturename_spec");
        if (prop_hairstyle_model_wig_spec)
        {
            auto prop_tex_spec = new_wig->mesh_property<std::string>("m:texturename_spec");
            prop_tex_spec[0] = prop_hairstyle_model_wig_spec[0];
        }

        // add texture to texture heap
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname_nm;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname_spec;
        texname = new_wig->get_mesh_property<std::string>("m:texturename");
        if (texname)
        {
            const std::string& texfilename = texname[0];
            //if texture already loaded, it should be on the heap
            gl::Texture* texture = gls->get_texture_from_heap(texfilename);
            //if not, load it
            if (texture == nullptr)
            {
                texture = new gl::Texture;
                if (texture->read(texfilename))
                {
                    //and add to texture heap
                    gls->add_texture_to_heap(texture);
                }
                else
                {
                    delete texture;
                }
            }
        }
        texname_nm = new_wig->get_mesh_property<std::string>("m:texturename_nm");
        if (texname_nm)
        {
            const std::string& texfilename = texname_nm[0];
            //if texture already loaded, it should be on the heap
            gl::Texture* texture = gls->get_texture_from_heap(texfilename);
            //if not, load it
            if (texture == nullptr)
            {
                texture = new gl::Texture;
                if (texture->read(texfilename))
                {
                    //and add to texture heap
                    gls->add_texture_to_heap(texture);
                }
                else
                {
                    delete texture;
                }
            }
        }
        texname_spec = new_wig->get_mesh_property<std::string>("m:texturename_spec");
        if (texname_spec)
        {
            const std::string& texfilename = texname_spec[0];
            //if texture already loaded, it should be on the heap
            gl::Texture* texture = gls->get_texture_from_heap(texfilename);
            //if not, load it
            if (texture == nullptr)
            {
                texture = new gl::Texture;
                if (texture->read(texfilename))
                {
                    //and add to texture heap
                    gls->add_texture_to_heap(texture);
                }
                else
                {
                    delete texture;
                }
            }
        }


        // adjust wig
        graphene::character::Surface_mesh_skin* current_blendshapes_base_mesh_skin = current_blendshapes_body.base(); // TODO HOCH ?!

        graphene::character::Blendshapes& hairstyle_model_body = *hairstyle_model.blendshapes()[0]; // TODO HOCH ?!
        graphene::character::Surface_mesh_skin* hairstyle_model_base_mesh_skin = hairstyle_model_body.base(); // TODO HOCH ?!

        std::vector<unsigned int> head_idx_vec;
        std::string filename_selection_head = dir_database_; filename_selection_head.append("/selections/head-6185.sel");
        if (!graphene::surface_mesh::load_selection_from_file(head_idx_vec, filename_selection_head))
        {
            std::cerr << "[DEBUG] Can't open selection for head." << std::endl;
            return false;
        }

        std::vector< Point > hairstyle_model_base_points;
        std::vector< Point > current_blendshapes_base_points;
        for ( unsigned int i = 0; i < head_idx_vec.size(); ++i)
        {
            graphene::surface_mesh::Surface_mesh::Vertex v(head_idx_vec[i]);

            hairstyle_model_base_points.push_back( hairstyle_model_base_mesh_skin->position(v) );
            current_blendshapes_base_points.push_back( current_blendshapes_base_mesh_skin->position(v) );
        }
        graphene::surface_mesh::RBF_deformer rbf_deformer(*new_wig, hairstyle_model_base_points, current_blendshapes_base_points);

        // slightly lift up
        for (auto v : new_wig->vertices())
        {
            new_wig->position(v)[1] += 0.0005;
        }
        new_wig->update_face_normals();
        new_wig->update_vertex_normals();

        // add skin to GL_state
        character_node_->add_skin(new_wig, gls);
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Character_generator::
set_haircolor(const Haircolor_type haircolor_type)
{
    std::cerr << "[DEBUG] in 'Character_generator::set_haircolor(...)' - A" << std::endl;

    if (!character_node_)
    {
        std::cerr << "[ERROR] Can't set haircolor. No character node set." << std::endl;
        return false;
    }

    if (Hairstyle_keep == latest_hairstyle_type_)
    {
        std::cout << "[WARNING] Can't set hair color. Set hairstyle first!" << std::endl;
        return true;
    }

    const bool has_wig = map_hairstyletype_2_wig_[latest_hairstyle_type_];

    if (!has_wig)
    {
        std::string filename_mask = dir_database_; filename_mask.append("/hair/masks/"); filename_mask.append(map_hairstyletype_2_mask_[latest_hairstyle_type_]);
        std::string filename_haircolor_template = dir_database_; filename_haircolor_template.append("/hair/"); filename_haircolor_template.append(map_hairstyle_and_colortype_2_filename_[latest_hairstyle_type_][haircolor_type]);

        cv::Mat mask_hairstyle_model = cv::imread(filename_mask.c_str(), 0);
        if(! mask_hairstyle_model.data )
        {
            std::cerr << "[ERROR] Can't set haircolor. Can't read mask!: " << filename_mask << std::endl;
            return false;
        }

        std::vector< graphene::gl::Texture* > textures_current_node = character_node_->get_textures();

        // modify albedo map only
        for (unsigned int i = 0; i < textures_current_node.size(); ++i)
        {
            if (textures_current_node[i]->name_.find("color.jpg") != std::string::npos)
            {
                if (textures_current_node[i]->name_.find("_wig_color.jpg") != std::string::npos)
                {
                    continue;
                }

                cv::Mat texture_opencv = texture_to_opencv(textures_current_node[i]);

                // change hair color
                graphene::utility::Texture_processing texture_processing;
                bool resized = false;

                cv::Mat img_haircolor_model = cv::imread(filename_haircolor_template.c_str());
                if(! img_haircolor_model.data )
                {
                    std::cerr << "[ERROR] Can't set hair color. Can't read albedo texture!" << std::endl;
                    return false;
                }

                // copy texture
                texture_processing.copy_and_paste(texture_opencv, img_haircolor_model, mask_hairstyle_model, resized);

                // cv::Mat result;
                // Super_fast_color_transfer color_transfer(img_haircolor_template_model, texture_opencv, result, mask_hairstyle_model); // TODO TODO TODO

                if (resized)
                {
                    const unsigned int new_w = texture_opencv.cols;
                    const unsigned int new_h = texture_opencv.rows;
                    textures_current_node[i]->width_  = new_w;
                    textures_current_node[i]->height_ = new_h;

                    textures_current_node[i]->data_.resize(new_w * new_h * 4);
                }

                opencv_to_texture(texture_opencv, textures_current_node[i]);

                character_node_->update_textures();
                break;
            }
        }
    }
    else
    {
        std::string filename_haircolor_template = dir_database_; filename_haircolor_template.append("/hair/"); filename_haircolor_template.append(map_hairstyle_and_colortype_2_filename_[latest_hairstyle_type_][haircolor_type]);

        cv::Mat img_haircolor_model = cv::imread(filename_haircolor_template.c_str());
        if(! img_haircolor_model.data )
        {
            std::cerr << "[ERROR] Can't set hair color. Can't read hair color template!" << std::endl;
            return false;
        }

        std::vector< graphene::gl::Texture* > textures_current_node = character_node_->get_textures();

        // modify albedo map of wig only
        for (unsigned int i = 0; i < textures_current_node.size(); ++i)
        {
            if (textures_current_node[i]->name_.find("_wig_color.jpg") != std::string::npos)
            {
                // change hair color
                opencv_to_texture(img_haircolor_model, textures_current_node[i]);

                character_node_->update_textures();
                break;
            }
        }        
    }

    return true;
}


//-----------------------------------------------------------------------------


const std::string
Character_generator::
get_full_filename_for_skintexture_color(const Skin_type skin_type) 
{
    std::string filename_skin_base = dir_database_; 
    filename_skin_base.append("/CG_skins/");
    filename_skin_base.append(map_skintype_2_filename_[skin_type]);
    std::string filename_skin      = filename_skin_base; 
    filename_skin.append("_color.jpg");

    return filename_skin;
}


//-----------------------------------------------------------------------------


void
Character_generator::
resolve_collisions(graphene::surface_mesh::Surface_mesh& clothes_model_mesh,
                   graphene::surface_mesh::Surface_mesh& reference_model_mesh,
                   const std::vector<size_t>& vertices_fix_deftrans_addclothes)
{
    // non-rigidly deform clothes model to reference model
    clothes_model_mesh.update_face_normals();
    clothes_model_mesh.update_vertex_normals();
    reference_model_mesh.update_face_normals();
    reference_model_mesh.update_vertex_normals();

    Surface_mesh::Vertex_property<bool> selected = clothes_model_mesh.vertex_property<bool>("v:selected");

    // set landmarks
    graphene::geometry::Point_set lm_ps;
    {
        if (vertices_fix_deftrans_addclothes.size() == 0)
        {
            std::cerr << "[ERROR] vertices_fix_deftrans_addclothes.size() == 0" << std::endl;
            exit(1);
        }

        std::vector<size_t> vertices_fix_boundary; // TODO WIRD SPÄTER NICHT (MEHR) GENUTZT
        // std::ofstream ofs;
        // ofs.open("/home/jachenba/test_boundary.sel");
        {
            for (auto v : clothes_model_mesh.vertices())
            {
                selected[v] = false;
            }

            for (unsigned int i = 0; i < vertices_fix_deftrans_addclothes.size(); ++i)
            {
                graphene::surface_mesh::Surface_mesh::Vertex v(vertices_fix_deftrans_addclothes[i]);
                selected[v] = true;
            }

            for (unsigned int i = 0; i < vertices_fix_deftrans_addclothes.size(); ++i)
            {
                graphene::surface_mesh::Surface_mesh::Vertex v(vertices_fix_deftrans_addclothes[i]);

                bool inner_point = true;
                for (auto vv : clothes_model_mesh.vertices(v))
                {
                    if (!selected[vv])
                    {
                        inner_point = false;
                        break;
                    }
                }
                if (!inner_point)
                {
                    vertices_fix_boundary.push_back( v.idx() );

                    // ofs << v.idx() << "\n";
                }
            }
        }

        Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks_mesh
            = clothes_model_mesh.mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
        std::vector<Surface_mesh::Vertex> &landmark_mesh_vec = landmarks_mesh[0];
        landmark_mesh_vec.clear();

        for (unsigned int i = 0; i < vertices_fix_deftrans_addclothes.size(); ++i)
        {
            graphene::surface_mesh::Surface_mesh::Vertex v(vertices_fix_deftrans_addclothes[i]);

            lm_ps.points_.push_back( reference_model_mesh.position(v) );
            lm_ps.landmarks_.push_back( i );

            landmark_mesh_vec.push_back( v );
        }
    }

    surface_mesh::Template_fit templatefit;
    surface_mesh::Energy& E = templatefit.energy();

    surface_mesh::Energy_term* e_lms     = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);
    surface_mesh::Energy_term* e_cpc     = E.energy_term_list().get_term(surface_mesh::ET_POINT_TO_POINT);
    surface_mesh::Energy_term* e_laplace = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE); // surface_mesh::ET_LAPLACIAN_COORDINATES
//            surface_mesh::Energy_term* e_laplace = E.energy_term_list().get_term(surface_mesh::ET_LAPLACIAN_COORDINATES);

    const double lms_weight = 1.0;
    e_lms->set_weight(lms_weight);
    e_lms->set_multiply_factor(1.0);

    const double cpc_weight = 1.0;
    e_cpc->set_weight(cpc_weight);
    e_cpc->set_multiply_factor(1.0);

    const double laplace_weight = 1e-7;
    e_laplace->set_weight(laplace_weight);
    e_laplace->set_multiply_factor(0.1);

    E.energy_term_list().set_term_for_termination_lambda(e_laplace->get_name());

    templatefit.set_template_mesh(&clothes_model_mesh);
    templatefit.set_point_set(&lm_ps);

    const double lambda = 1e-7;
    double current_lambda;
    std::vector<surface_mesh::Correspondence>& projected_corrs = E.energy_term_data().correspondences;
    Triangle_kD_tree reference_model_mesh_kd(reference_model_mesh, 100, 60);
    auto ref_mesh_points  = reference_model_mesh.vertex_property<Point>("v:point");
    auto ref_mesh_normals = reference_model_mesh.vertex_property<Normal>("v:normal");
    double error_total_previous = DBL_MAX;
    double error_total_current  = 0.0;
    const double conv_val       = 0.005;
    double conv_val_rel         = 0.0;
    const unsigned int max_iter = 25;
    do
    {
        templatefit.update_restpose();

        unsigned int iter_second_loop = 0;
        bool second_loop_done = false;
        while (!second_loop_done)
        {
            iter_second_loop++;
            std::cerr << "[DEBUG] iter_second_loop: " << iter_second_loop << std::endl;

            // compute correspondences / collision constraints
            projected_corrs.clear();
            // std::ofstream ofs;
            // ofs.open("/home/jachenba/test_collision.sel");
            bool any_collision = false;
            for (auto v : clothes_model_mesh.vertices())
            {
                if (selected[v]) // no collision handling for selected, i.e., non-clothes vertices
                {
                    continue;
                }

                const auto nn_tri = reference_model_mesh_kd.nearest( clothes_model_mesh.position(v) );
                const Vec3d nearest_on_tri(nn_tri.nearest[0],nn_tri.nearest[1],nn_tri.nearest[2]);
                const double dist_nearest_on_tri = nn_tri.dist;
                Vec3d diff_vec = nearest_on_tri - Vec3d(clothes_model_mesh.position(v));
                diff_vec.normalize();
                // if (norm(diff_vec) < 1e-9)
                // {
                //     continue;
                // }
                auto fvit = reference_model_mesh.vertices(nn_tri.face);
                const auto v0 = *fvit;
                const auto v1 = *(++fvit);
                const auto v2 = *(++fvit);
                const Point p0 = ref_mesh_points[v0];
                const Point p1 = ref_mesh_points[v1];
                const Point p2 = ref_mesh_points[v2];
                const Normal n0 = ref_mesh_normals[v0];
                const Normal n1 = ref_mesh_normals[v1];
                const Normal n2 = ref_mesh_normals[v2];
                // get barycentric coordinates
                const Point b = graphene::geometry::barycentric_coordinates(nn_tri.nearest, p0, p1, p2);
                Vec3d n_on_ref_mesh = b[0]*n0 + b[1]*n1 + b[2]*n2;
                n_on_ref_mesh.normalize();
                bool collision = false;
                if (dot(n_on_ref_mesh, diff_vec) > 0)
                {
                    collision = true;
                    any_collision = true;
                }
                const double eps = 0.002;
                if (dist_nearest_on_tri < eps)
                {
                    collision = true;
                    any_collision = true;
                }
                // if collision, resolve and add CPC
                if (collision)
                {
                    // ofs << v.idx() << "\n";

                    surface_mesh::Correspondence c;
                    c.constr_dir = surface_mesh::corresp_dir_mesh2ps;
                    c.weight = 1.0;
                    c.on_ps = nearest_on_tri + eps*n_on_ref_mesh;
                    // c.on_ps = nearest_on_tri + 0.002*diff_vec;
                    c.on_template_vh = v;
                    projected_corrs.push_back(c);
                }
                else
                {
                    surface_mesh::Correspondence c;
                    c.constr_dir = surface_mesh::corresp_dir_mesh2ps;
                    c.weight = 0.1;
                    c.on_ps = clothes_model_mesh.position(v);
                    c.on_template_vh = v;
                    projected_corrs.push_back(c);
                }
            }

            // sum correspondences
            double& sum_constr_weights = E.energy_term_data().sum_constr_weights;
            sum_constr_weights = 0.0;
            for (auto c : E.energy_term_data().correspondences)
            {
                sum_constr_weights += c.weight;
            }
            if ( !any_collision || sum_constr_weights < 1e-5 )
            {
                std::cerr << "[DEBUG] no collision" << std::endl;
                break; // no collision
            }

            E.minimize(0.01);

            // break;

            // error_total_current = E.get_function_value();
            error_total_current = E.energy_term_list().evaluate();

            std::cerr << "[DEBUG]                         error_total_current: " << error_total_current << std::endl;

            conv_val_rel = std::abs(error_total_current - error_total_previous) / error_total_previous;
            std::cerr << "[DEBUG]                         conv_val_rel: " << conv_val_rel << std::endl;
            if (conv_val_rel < conv_val)
            {
                second_loop_done = true;
            }
            error_total_previous = error_total_current;

            if (iter_second_loop >= max_iter)
            {
                second_loop_done = true;
            }
        }

        E.energy_term_list().apply_weight_multipliers();
        current_lambda = E.energy_term_list().get_termination_lambda();
    }
    while (current_lambda >= lambda);

    for (auto v : clothes_model_mesh.vertices())
    {
        selected[v] = false;
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
