//=============================================================================
#ifndef GRAPHENE_TEMPLATE_FIT_H
#define GRAPHENE_TEMPLATE_FIT_H
//=============================================================================

//== INCLUDES =================================================================


#include <string>
#include <tuple>

#include <graphene/character/data_structure/Character.h>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/geometry/Point_set.h>
#include <graphene/geometry/Bounding_box.h>

#include "my_types.h"
#include "Landmarks_manager.h"
#include "Energy.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


enum Transformation_direction { trans_dir_mesh2ps, trans_dir_ps2mesh };


class Template_fit
{
private: //------------------------------------------------------- private data

    Energy                              energy_;

    graphene::geometry::Point_set*      point_set_;             // the actual point set (possibly filtered / possibly subsampled / possibly transformed)
    Surface_mesh*                       template_mesh_;

public: //--------------------------------------------------------- public data



public: //---------------------------------------------------- public functions

    // constructor
    Template_fit();

    // clear
    void clear();

    Energy& energy() { return energy_; }


//--------------------------------------------------------------- loading stuff


    void set_point_set(graphene::geometry::Point_set* point_set);
    void set_template_mesh(Surface_mesh* template_mesh);
    bool load_selection_to_omit(const std::string& filename);
    bool load_selection_to_omit(const std::string& filename,
                                const std::string& filename_mapping);
    void clear_selection_to_omit();
    bool load_selection_to_keep(const std::string& filename);
    bool load_selection_to_keep(const std::string& filename, const std::string& filename_mapping, const bool add = false);
    void clear_selection_to_keep();
    bool load_selection_ground(const std::string& filename);
    void clear_selection_ground();
    bool load_mouth_shut_selection(const std::string& filename);
    bool load_mouth_shut_selection(const std::string& filename, const std::string& filename_mapping);

    bool load_ear_landmarks_template(const std::string& filename);
    bool load_ear_landmarks_template(const std::string& filename, const std::string& filename_mapping);
    bool load_ear_landmarks_pointset(const std::string& filename);

    bool load_vertexweighting(const std::string& filename, const std::string& property_name, bool multiply=true);
    bool load_vertexweighting(const std::string& filename, const std::string& filename_mapping, const std::string& property_name, bool multiply=true);
    bool clear_vertexweighting(const std::string& property_name);


//---------------------------- initial transformations based on landmarks


    // transforms 'mesh 2 pointset' OR 'pointset 2 mesh'
    bool do_similarity_transformation_lm(const Transformation_direction trans_dir);

    bool do_similarity_transformation_cpc(const Transformation_direction trans_dir,
                                          const Correspondences_settings& constr_settings,
                                          double& relative_change);

//-------------------------------------------------------------- nonrigid stuff

    void start(const Correspondences_settings& constr_settings,
               const unsigned int max_iter,
               const double conv_mean_diff,
               const double until_lambda,
               const bool make_plastic = true,
               const bool no_decrease  = false
               );


//-------------------------------------------------------------- misc/testing stuff


    void update_restpose();
    void write_mesh_backtransformed( const std::string& filename_mesh );
    double evaluate_fitting();

//---------------------------------------------------------------------- getter


    geometry::Point_set& get_point_set()     { return *point_set_; }
    Surface_mesh&        get_template_mesh() { return *template_mesh_; }

    Landmarks_manager&   get_landmarks_manager() { return energy_.energy_term_data().landmarks_manager; }

    PCA&                 get_pca() { return energy_.energy_term_data().pca; }
private: //-------------------------------------------------- private functions

    void sum_correspondences(const std::vector<Correspondence>& correspondences);

    void update_area_property();
    void update_cotan_property();
    void update_laplaciancoordinate_property();
    void update_aniso_laplace_properties();


    void transform_points(const Mat4f& M);

    void clean_up_mesh_properties();

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_TEMPLATE_FIT_H
//=============================================================================
