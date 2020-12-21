//=============================================================================
#ifndef GRAPHENE_LOGGING_SETTING_H
#define GRAPHENE_LOGGING_SETTING_H
//=============================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>



//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================


struct Logging_settings
{
    bool         log_etotal_over_iter = false;
    std::string  log_etotal_over_iter_filename = "/tmp/tmp.dat";

    bool         log_iter_over_lgs = false;
    std::string  log_iter_over_lgs_filename = "/tmp/tmp.dat";

    bool         log_iter_lgs_solved = false;
    std::string  log_iter_lgs_solved_filename = "/tmp/tmp.dat";

    bool         write_intermediate_meshes     = false;
    std::string  write_intermediate_meshes_dir = "/tmp/";
    bool         write_transformed_to_orig_ps  = true;
    bool         write_transformed_to_template = true;

    bool         write_eyes_meshes             = true;

    bool         write_target_contour_points   = true;

    bool         write_eyes_ps                 = true;

    bool         log_avg_iterations_for_minimization          = false;
    std::string  log_avg_iterations_for_minimization_filename = "/tmp/tmp.dat";

    bool         log_time_overall = false;
    std::string  log_time_overall_filename = "/tmp/tmp.dat";

    // log times (w.r.t. certain stiffnesses): 1) compute correspondences 2) setup linear systems 3) minimization
    bool         log_time_details = false;
    std::string  log_time_details_filename = "/tmp/tmp.dat";

    // log number of minimizations (w.r.t. certain stiffnesses)
    bool         log_iter_minimizations = false;
    std::string  log_iter_minimizations_filename = "/tmp/tmp.dat";
};

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_LOGGING_SETTING_H
//=============================================================================
