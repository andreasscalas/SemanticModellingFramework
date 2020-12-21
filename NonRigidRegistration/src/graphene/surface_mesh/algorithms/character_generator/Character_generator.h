//=============================================================================
#ifndef GRAPHENE_CHARACTER_GENERATOR_H
#define GRAPHENE_CHARACTER_GENERATOR_H
//=============================================================================

//== INCLUDES =================================================================


#include <string>
#include <map>
#include <fstream>

#include <graphene/character/scene_graph/Character_node.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::scene_graph::Character_node;


//== CLASS DEFINITION =========================================================


enum Model_type { Model_keep,
                  Model_backup,
                  Model_custom_M_young,
                  Model_custom_W_young,
                  Model_custom_M_midyoung,
                  Model_custom_W_midyoung,
                  Model_custom_M_midold,
                  Model_custom_W_midold,
                  Model_custom_M_old,
                  Model_custom_W_old,
                  Model_M_Anthony,
                  Model_M_Asaiah,
                  Model_M_Biff,
                  Model_M_Buck,
                  Model_M_Cody,
                  Model_M_David,
                  Model_M_Dylan,
                  Model_M_Elias,
                  Model_M_Evan,
                  Model_M_Gabriel,
                  Model_M_Gilbert,
                  Model_M_Huey,
                  Model_M_Jack,
                  Model_M_Jacob,
                  Model_M_Jamal,
                  Model_M_JeanRene,
                  Model_M_Jeremyah,
                  Model_M_Johnny,
                  Model_M_Jose,
                  Model_M_Kevin,
                  Model_M_Kurt,
                  Model_M_Luke,
                  Model_M_Maurice,
                  Model_M_Minh,
                  Model_M_Nathan,
                  Model_M_Ralph,
                  Model_M_Ryan,
                  Model_M_Schwartz,
                  Model_M_Sean,
                  Model_M_Stephan,
                  Model_M_Taylor,
                  Model_M_Terence,
                  Model_M_Timothy,
                  Model_M_Todd,
                  Model_M_Victor,
                  Model_M_Warren,
                  Model_M_William,
                  Model_M_Wyatt,
                  Model_W_Adrienne,
                  Model_W_Amy,
                  Model_W_Ashley,
                  Model_W_Barbara,
                  Model_W_Brooke,
                  Model_W_Cathy,
                  Model_W_Charley,
                  Model_W_Chloe,
                  Model_W_Claire,
                  Model_W_Claudia,
                  Model_W_Daniela,
                  Model_W_Debbie,
                  Model_W_Eleanor,
                  Model_W_Elsa,
                  Model_W_Esmeralda,
                  Model_W_Evelyn,
                  Model_W_Grace,
                  Model_W_Haley,
                  Model_W_Jada,
                  Model_W_Jeanne,
                  Model_W_Jenna,
                  Model_W_Kari,
                  Model_W_Kate,
                  Model_W_Katherine,
                  Model_W_Kim,
                  Model_W_Lena,
                  Model_W_Lily,
                  Model_W_Makayla,
                  Model_W_Maya,
                  Model_W_Mia,
                  Model_W_Morgan,
                  Model_W_Nicole,
                  Model_W_Paige,
                  Model_W_Pam,
                  Model_W_Rebecca,
                  Model_W_Silke,
                  Model_W_Sophia,
                  Model_W_Tiffany,
                  Model_W_Trinity,
                  Model_W_Venus,
                  Model_W_Wanda,
                  Model_W_Yoki,
                  Model_W_Zoe,
                  Model_W_Zyra };


enum Skin_type { Skin_keep, // 0
                 Skin_M_01,
                 Skin_M_02,
                 Skin_M_03,
                 Skin_M_04,
                 Skin_M_05,
                 Skin_M_06,
                 Skin_M_07,
                 Skin_M_08,
                 Skin_M_09,                 
                 Skin_M_10, // 10
                 Skin_M_11,
                 Skin_M_12,
                 Skin_M_16,
                 Skin_M_17,
                 Skin_M_18,
                 Skin_M_19,
                 Skin_M_20,
                 Skin_M_21,
                 Skin_M_22,                 
                 Skin_M_23, // 20
                 Skin_M_24,
                 Skin_M_25,
                 Skin_M_27,
                 Skin_M_28,
                 Skin_M_29,
                 Skin_M_30,
                 Skin_M_31,
                 Skin_M_32,
                 Skin_M_33,
                 Skin_M_36, // 30
                 Skin_M_37,
                 Skin_M_38,
                 Skin_M_39,
                 Skin_M_40,
                 Skin_M_41,
                 Skin_M_44,
                 Skin_M_45,
                 Skin_M_46,
                 Skin_M_47,
                 Skin_M_48, // 40
                 Skin_M_49,
                 Skin_W_01,
                 Skin_W_02,
                 Skin_W_03,
                 Skin_W_04,
                 Skin_W_05,
                 Skin_W_06,
                 Skin_W_07,
                 Skin_W_08,
                 Skin_W_09, // 50
                 Skin_W_10,
                 Skin_W_11,
                 Skin_W_12,
                 Skin_W_13,
                 Skin_W_14,
                 Skin_W_15,
                 Skin_W_17,
                 Skin_W_19,
                 Skin_W_20,
                 Skin_W_21, // 60
                 Skin_W_22,
                 Skin_W_23,
                 Skin_W_24,
                 Skin_W_25,
                 Skin_W_26,
                 Skin_W_27,
                 Skin_W_28,
                 Skin_W_29,
                 Skin_W_30,
                 Skin_W_31, // 70
                 Skin_W_32,
                 Skin_W_33,
                 Skin_W_34,
                 Skin_W_35,
                 Skin_W_37,
                 Skin_W_38,
                 Skin_W_39,
                 Skin_W_40,
                 Skin_W_44,
                 Skin_W_45, // 80
                 Skin_W_46,
                 Skin_W_47,
                 Skin_W_48,
                 Skin_W_49,
                 Skin_LAST };


enum Eyecolor_type { Eyecolor_keep,
                     Eyecolor_01,
                     Eyecolor_02,
                     Eyecolor_03,
                     Eyecolor_04,
                     Eyecolor_05,
                     Eyecolor_06,
                     Eyecolor_07,
                     Eyecolor_08,
                     Eyecolor_09,
                     Eyecolor_10,
                     Eyecolor_11,
                     Eyecolor_12,
                     Eyecolor_13,
                     Eyecolor_14,
                     Eyecolor_15,
                     Eyecolor_16,
                     Eyecolor_17,
                     Eyecolor_18,
                     Eyecolor_19,
                     Eyecolor_20,
                     Eyecolor_21,
                     Eyecolor_22,
                     Eyecolor_23,
                     Eyecolor_24,
                     Eyecolor_25,
                     Eyecolor_26,
                     Eyecolor_27,
                     Eyecolor_28,
                     Eyecolor_29,
                     Eyecolor_30,
                     Eyecolor_31,
                     Eyecolor_32,
                     Eyecolor_33,
                     Eyecolor_34 };


enum Clothes_type { Clothes_keep,
                    Clothes_custom_casual_M,
                    Clothes_custom_formal_M,
                    Clothes_custom_formal_W,
                    Clothes_custom_sport_M,
                    Clothes_custom_kitchen_M,
                    Clothes_self_jascha_casual_greensuit,
                    Clothes_self_jascha_longsleeve_greensuit,
                    Clothes_self_jascha_tshirt_greensuit,
                    Clothes_self_jascha_jeansandshoes_greensuit,
                    Clothes_self_teresa_casual_greensuit,
                    Clothes_self_teresa_jeans_greensuit,
                    Clothes_self_teresa_top_greensuit,
                    Clothes_self_christina_casual_manual,
                    Clothes_self_andreas_casual_manual,
                    Clothes_self_matthias_casual_manual };


enum Hairstyle_type { Hairstyle_keep,
                      Hairstyle_M_02,
                      Hairstyle_M_03,
                      Hairstyle_M_07,
                      Hairstyle_M_09,
                      Hairstyle_M_17,
                      Hairstyle_W_02,
                      Hairstyle_W_05,
                      Hairstyle_W_07,
                      Hairstyle_W_10,
                      Hairstyle_W_12,
                      Hairstyle_W_19,
                      Hairstyle_W_24,
                      Hairstyle_W_28,
                      Hairstyle_W_33,
                      Hairstyle_W_34,
                      Hairstyle_W_35 };


enum Haircolor_type { Haircolor_blond,
                      Haircolor_brown,
                      Haircolor_gray };


class Character_generator
{
public: //---------------------------------------------------- public functions

    // constructor
    Character_generator();

    // constructor
    Character_generator(const std::string dir_database);

    bool set_dir_database(const std::string dir_database);

    bool set_character_node(Character_node* character_node);

    std::string get_model_filename(const Model_type model_type);

    bool set_skin(const Skin_type skin_type);

    bool set_eyecolor(const Eyecolor_type eyecolor_type);

    bool add_clothes(const Clothes_type clothes_type, gl::GL_state* gls, const double chef_hat_scale, const double hair_offset, const double hair_offset2, const std::string filename_selection_subdivision);

    bool add_hairstyle(const Hairstyle_type hairstyle_type, gl::GL_state* gls, const std::string filename_selection_subdivision);

    bool set_haircolor(const Haircolor_type haircolor_type);

    const std::string get_full_filename_for_skintexture_color(const Skin_type skin_type) ;


private:

    void init();

    void resolve_collisions(graphene::surface_mesh::Surface_mesh& clothes_model_mesh,
                            graphene::surface_mesh::Surface_mesh& reference_model_mesh,
                            const std::vector<size_t>& vertices_fix_deftrans_addclothes);

    Surface_mesh character_to_surface_mesh(graphene::character::Character& character)
    {
        graphene::character::Blendshapes* blendshapes_body = character.blendshapes()[0];
        Surface_mesh result = *blendshapes_body->base();

        return result;
    }

    void load_selection_from_file(graphene::surface_mesh::Surface_mesh& mesh, const std::string filename_selection)
    {
        Surface_mesh::Vertex_property<bool> selected = mesh.vertex_property<bool>("v:selected", false);
        for (auto v : mesh.vertices())
        {
            selected[v] = false;
        }

        std::ifstream ifs( filename_selection.c_str() );
        unsigned int idx;
        while (true)
        {
            ifs >> idx;
            if (ifs.eof()) break;
            selected[Surface_mesh::Vertex(idx)] = true;
        }
        ifs.close();
    }


private:

    Hairstyle_type                         latest_hairstyle_type_;

    std::string                            dir_database_;

    Character_node*                        character_node_;

    std::map<Model_type, std::string>      map_modeltype_2_filename_;

    std::map<Skin_type, std::string>       map_skintype_2_filename_;

    std::map<Eyecolor_type, std::string>   map_eyecolortype_2_filename_;

    std::map<Clothes_type, std::string>    map_clothestype_2_filename_;
    std::map<Clothes_type, std::string>    map_clothestype_2_mask_;
    std::map<Clothes_type, std::string>    map_clothestype_2_referencemodel_;
    std::map<Clothes_type, std::string>    map_clothestype_2_fixdeftrans_;
    std::map<Clothes_type, bool>           map_clothestype_2_selfscanned_;

    std::map<Hairstyle_type, std::string>  map_hairstyletype_2_filename_;

    std::map<Hairstyle_type, std::string>  map_hairstyletype_2_mask_;

    std::map<Hairstyle_type, bool>         map_hairstyletype_2_wig_;

    std::map< Hairstyle_type, std::map<Haircolor_type, std::string> >  map_hairstyle_and_colortype_2_filename_;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_CHARACTER_GENERATOR_H
//=============================================================================
