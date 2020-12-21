//=============================================================================


#ifndef GRAPHENE_ENERGY_TERM_DATA_H
#define GRAPHENE_ENERGY_TERM_DATA_H


//== INCLUDES =================================================================

#include <graphene/surface_mesh/algorithms/templatefitting/my_types.h>
#include <graphene/surface_mesh/algorithms/pca/PCA_opencv.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Landmarks_manager.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


struct Column_offsets
{
    unsigned int  offset_match       = 0;
    unsigned int  offset_nrd_w_rigid = 0;
    unsigned int  offset_nrd_w_pca   = 0;
    unsigned int  offset_nrd_w_arap  = 0;

    void clear()
    {
        offset_match = offset_nrd_w_rigid = offset_nrd_w_pca = offset_nrd_w_arap = 0;
    }
};


class
Energy_term_data
{
public:

    Energy_term_data() :
        sum_constr_weights(0.0),
        surface_area_aniso(0.0),
        surface_area(0.0),
        template_mesh(NULL),
        point_set(NULL)
    {}

    ~Energy_term_data() {}


public:

    Column_offsets              column_offsets;
    std::vector<Correspondence> correspondences;
    double                      sum_constr_weights;
    Landmarks_manager           landmarks_manager;
    PCA_opencv                  pca;
    double                      surface_area_aniso;
    double                      surface_area;
    Surface_mesh*               template_mesh;
    geometry::Point_set*        point_set;

    void clear()
    {
        column_offsets.clear();
        correspondences.clear();
        sum_constr_weights = 0.0;
        landmarks_manager.clear();
        pca.clear();
        surface_area = 0.0;
        surface_area_aniso = 0.0;
        template_mesh = NULL;
        point_set = NULL;
    }

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_ENERGY_TERM_DATA_H
//=============================================================================
