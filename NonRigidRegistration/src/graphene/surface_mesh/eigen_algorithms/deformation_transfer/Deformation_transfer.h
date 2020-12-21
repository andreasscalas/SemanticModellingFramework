//=============================================================================
#ifndef GRAPHENE_DEFORMATION_TRANSFER_H
#define GRAPHENE_DEFORMATION_TRANSFER_H
//=============================================================================

//== INCLUDES =================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <Eigen/Sparse>

#define SparseSPDSolver SimplicialLDLT

#include <set>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


/// \brief enum for setting the deformation transfer method.
enum dt_method
{
    BOTSCH = 0,
    SUMNER = 1,
    EXAMPLE_BASED = 2,
};

enum dt_direction
{
    EXPRESSION_TRANSFER = 0,
    SHAPE_TRANSFER = 1,
};


/// \brief Setting up deformation transfer, example based facial rigging or similar methods requires many \
/// meshes and other information. Rather than passing several variable within a function, we capsule all the\
/// data and leave the methods decide, what they need. If our input data is insufficient, the method is responsible\
/// to notify the user with a meaningful error message.
struct Deformation_transfer_data
{
    std::string source_undeformed;
    std::string target_undeformed;
    std::set<std::string> source_deformed;
    std::set<std::string> source_deformed_names;
    std::string location_target_deformed;
    std::set<std::string> examples;
    std::vector<std::vector<float>> examples_weights;
    dt_method method;
    dt_direction direction;
    bool ex_weight_estimation;
};


//== CLASS DEFINITION =========================================================
/// \brief an abstract class for better combining of different Defomation transfer classes
///  using the Template method pattern (https://en.wikipedia.org/wiki/Template_method_pattern).

/// usage:
/// Deformation_transfer_child df_c();
/// deform_target( .... );

class Deformation_transfer
{
public:

    // declares a column-major sparse matrix type of double
    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double>      Tripl;

    // destructor
    virtual ~Deformation_transfer();

    // do deformation transfer by specifying the location of the input and output files
    void create_deformed_targets(const Deformation_transfer_data& data);

    // do deformation transfer by specifying the undeformed and deformed source mesh as well as the undeformed target mesh
    bool create_deformed_targets(Surface_mesh& source_undeformed,
                                 Surface_mesh& source_deformed,
                                 Surface_mesh& target_undeformed);


protected:

    /// \brief Constructor of an abstract class. This class is not intended to be used directly.
    /// Use instead its child classes.
    Deformation_transfer();

    /// function that marks non-moving vertices as locked
    void setup_locked_vert(Surface_mesh& src_neutral_mesh,
                           const std::set<std::string>& files_src_expr,
                           Surface_mesh& tar_neutral_mesh);

    /// function that marks non-moving vertices as locked
    void setup_locked_vert(Surface_mesh& src_neutral_mesh,
                           Surface_mesh& src_expr_mesh,
                           Surface_mesh& tar_neutral_mesh);

    /// compute deltas
    void compute_deltas(Surface_mesh& src_neutral_mesh,
                        Surface_mesh& src_expr_mesh);

    /// lock vertices with small delta
    void lock_vertices_small_delta(Surface_mesh& src_neutral_mesh,
                                   Surface_mesh& tar_neutral_mesh);

    /// creates a prefactorized laplacian matrix from the undeformed target mesh
    virtual void prefactorize(Surface_mesh& mesh_target_undeformed, unsigned int& mat_size) = 0;

    /// transfers deformation of source deformed to target deformed
    virtual bool create_deformed_target(Surface_mesh& mesh_source_undeformed,
                                        Surface_mesh& mesh_source_deformed,
                                        Surface_mesh& mesh_target_undeformed,
                                        Surface_mesh& mesh_target_deformed,
                                        unsigned int mat_size) = 0;

    /// Eigent Solver for prefactorization
    Eigen::SparseSPDSolver< SpMat >* solver_;

    /// laplacian Matrix
    SpMat* A_;

    /// state of the factorization
    bool factorization_state;

};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_DEFORMATION_TRANSFER_H
//=============================================================================
