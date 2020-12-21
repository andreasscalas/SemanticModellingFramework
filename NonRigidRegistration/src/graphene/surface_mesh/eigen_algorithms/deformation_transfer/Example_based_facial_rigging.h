//=============================================================================
#ifndef GRAPHENE_EXAMPLE_BASED_FACIAL_RIGGING_H
#define GRAPHENE_EXAMPLE_BASED_FACIAL_RIGGING_H
//=============================================================================

//== INCLUDES =================================================================


#include "Deformation_transfer.h"
#include <Eigen/LU>
#include <Eigen/Geometry>

//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class Example_based_facial_rigging:public Deformation_transfer
{


public:
    // constructor
   Example_based_facial_rigging(){};

    // destructor
    ~Example_based_facial_rigging();

    void create_deformed_targets(const Deformation_transfer_data& data);

protected:

    // generate blendshapes
   void generate_blendshapes(const Deformation_transfer_data& data);

    void check_number_vertices( const Surface_mesh& mesh ) const;

    /// creates a prefactorized laplacian matrix from the undeformed target mesh
    virtual void prefactorize(Surface_mesh& mesh_target_undeformed, unsigned int& mat_size){};

    /// transfers deformation of source deformed to target deformed
    virtual bool create_deformed_target(Surface_mesh& mesh_source_undeformed,
       Surface_mesh& mesh_source_deformed,
       Surface_mesh& mesh_target_undeformed,
       Surface_mesh& mesh_target_deformed,
       unsigned int mat_size){return false;};

    unsigned int num_vertices_;

    // B_0
    Surface_mesh                         mesh_bs_B0_;   
    Surface_mesh                         generic_bs_mesh_A0_;    

    std::vector< Surface_mesh >          generic_bs_meshes_Ai_;    
    std::vector< Surface_mesh >          example_meshes_Sj_;

    Eigen::MatrixXd                      coeffs_aij_;
};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_EXAMPLE_BASED_FACIAL_RIGGING_H
//=============================================================================
