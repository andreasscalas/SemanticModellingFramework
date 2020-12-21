//=============================================================================
#ifndef GRAPHENE_CORRESPONDENCES_SETTING_H
#define GRAPHENE_CORRESPONDENCES_SETTING_H
//=============================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/surface_mesh/algorithms/templatefitting/types/Correspondence.h>



//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================


struct Correspondences_settings
{
    bool                 filter_dist;
    double               dist_until; // in dm

    bool                 filter_normals;
    double               normal_until;

    bool                 std_dev_filter;
    double               std_dev_factor;

    bool                 worst_nPerc_filter;
    double               nPerc_worst;

    bool                 boundary_filter;

    bool                 symmetry_filter;
    double               symmetry_epsilon;

    bool                 w_distance_1r;
    bool                 w_distance_1ddMax;
    bool                 w_dot_nn;
    bool                 w_distance_huber;
    double               w_distance_huber_value;

    Correspondence_direction constr_dir;

    Correspondences_settings() :
        filter_dist(false),
        dist_until(0.01),
        filter_normals(false),
        normal_until(30.0),
        std_dev_filter(false),
        std_dev_factor(0.1),
        worst_nPerc_filter(false),
        nPerc_worst(5.0),
        boundary_filter(false),
        symmetry_filter(false),
        symmetry_epsilon(0.01),
        w_distance_1r(false),
        w_distance_1ddMax(false),
        w_dot_nn(false),
        w_distance_huber(false),
        w_distance_huber_value(0.1),
        constr_dir(corresp_dir_ps2mesh)
    {}
};



//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_CORRESPONDENCES_SETTING_H
//=============================================================================
