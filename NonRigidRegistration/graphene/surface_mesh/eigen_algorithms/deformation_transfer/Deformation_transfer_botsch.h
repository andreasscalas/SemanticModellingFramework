//=============================================================================
#ifndef GRAPHENE_DEFORMATION_TRANSFER_BOTSCH_H
#define GRAPHENE_DEFORMATION_TRANSFER_BOTSCH_H
//=============================================================================

//== INCLUDES =================================================================


#include "Deformation_transfer.h"


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


//== CLASS DEFINITION =========================================================


class Deformation_transfer_botsch : public Deformation_transfer
{

    // declares a column-major sparse matrix type of double
    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double>      Tripl;

public:

    /// constructor - only calls parent constructor
    Deformation_transfer_botsch();

    // destructor - implicit call of parent destructor
    ~Deformation_transfer_botsch();


protected:

    /// transfers the deformation from source deformed to target deformed
    bool create_deformed_target(Surface_mesh& mesh_source_undeformed,
                                Surface_mesh& mesh_source_deformed,
                                Surface_mesh& mesh_target_undeformed,
                                Surface_mesh& mesh_target_deformed,
                                unsigned int mat_size);

    /// creates a prefactorized laplacian matrix from the undeformed target mesh
    void prefactorize(Surface_mesh& mesh_target_undeformed, unsigned int& mat_size);

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_DEFORMATION_TRANSFER_BOTSCH_H
//=============================================================================
