//=============================================================================
#ifndef GRAPHENE_DEFORMATION_TRANSFER_SUMNER_H
#define GRAPHENE_DEFORMATION_TRANSFER_SUMNER_H
//=============================================================================

//== INCLUDES =================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/geometry/Vector.h>

#include <vector>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::surface_mesh::Surface_mesh;
using graphene::Point;


//== CLASS DEFINITION =========================================================

class Deformation_transfer_sumner
{
public:

    // constructor
    Deformation_transfer_sumner( Surface_mesh& mesh_target_undeformed );

    // destructor
    ~Deformation_transfer_sumner();

    // perform deformation transfer
    bool deform_target( const std::vector<Point> &mesh_source_undeformed,
                        const std::vector<Point> &mesh_source_deformed );

    Surface_mesh get_deformed_target();


private:

    Surface_mesh mesh_result_;


};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_DEFORMATION_TRANSFER_SUMNER_H
//=============================================================================
