//=============================================================================


#ifndef GRAPHENE_NONLINEAR_DISCRETE_SHELL_H
#define GRAPHENE_NONLINEAR_DISCRETE_SHELL_H


//== INCLUDES =================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/types.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


//== DOXYGEN ==================================================================


/// \addtogroup surface_mesh
/// @{


//== CLASS DEFINITION =========================================================


enum Nonlin_discrete_shell_mode { nonlin_mesh_ik,
                                  nonlin_interpolation,
                                  nonlin_deformation_transfer };


class Nonlinear_discrete_shell
{

    typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
    typedef Eigen::Triplet<double>      Tripl;

public:

    // constructor
    Nonlinear_discrete_shell();

    // destructor
    ~Nonlinear_discrete_shell();

    // set base mesh
    bool set_base_mesh(Surface_mesh& base_mesh);

    bool set_stiffness(const double stiffness_length, const double stiffness_angle);

    inline const int get_num_examples() const { return num_examples_; };

    bool add_example(const std::string filename);

    std::string next();

    std::string back_to_base();

    bool mesh_ik(unsigned int max_iters);

    bool interpolate(std::vector<double> weights);

    bool deformation_transfer(std::string filename_src_undeformed, std::string filename_src_deformed);
    bool deformation_transfer(Surface_mesh& mesh_src_undeformed, Surface_mesh& mesh_src_deformed);


private: //-------------------------------------------------- private functions

    // deform
    bool optimize(unsigned int max_iters = 200);

    void compute_length_and_angle(Surface_mesh& mesh);

    bool compute_weights_of_length_and_angle(const double stiffness_length, const double stiffness_angle);

    inline const double length(const Point& p0, const Point& p1) const;
    inline const double area(const Point& p0, const Point& p1, const Point& p2) const;
    inline const double angle(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const;

    inline void gradients_length(const Point& p0, const Point& p1, Vec3d& g0, Vec3d& g1) const;
    inline void gradients_angle(const Point& p0, const Point& p1, const Point& p2, const Point& p3, Vec3d& g0, Vec3d& g1, Vec3d& g2, Vec3d& g3) const;

    const double error();

    // collect free edges and free vertices
    bool update_free_vertices_and_edges();

    int&    idx    (const Surface_mesh::Vertex vh) { return vidx_[vh]; }
    Point&  new_pos(const Surface_mesh::Vertex vh) { return potential_position_[vh]; }

    inline const double weight_length(const Surface_mesh::Edge eh) const
    {
        return weight_edge_length_[eh];
    }

    inline const double weight_angle(const Surface_mesh::Edge eh)
    {
        for(int i = 0; i < num_examples_; i++)
        {
            auto adiff_example = example_meshes_[i].edge_property<double>("e:adiff");
            if(adiff_example[eh] > 3.141593)
            {
                return 0.0;
            }
        }

        return weight_edge_angle_[eh];
    }

    double target_length(const Surface_mesh::Edge eh);
    double target_angle(const Surface_mesh::Edge eh);


private: //------------------------------------------------------- private data

    Surface_mesh*                current_mesh_;
    Surface_mesh                 base_mesh_;
    std::vector< Surface_mesh >  example_meshes_;

    Surface_mesh::Vertex_property<Point>   deformed_points_backup_;

    // edge weights
    Surface_mesh::Edge_property<double>    weight_edge_length_;
    Surface_mesh::Edge_property<double>    weight_edge_angle_;

    // vertex handle of mesh -> idx of free vertex (-1 iff not free)
    Surface_mesh::Vertex_property<int>     vidx_;

    // potential position while gauss newton iterations
    Surface_mesh::Vertex_property<Point>   potential_position_;

    // interpolation weights
    std::vector<double> model_weight_;

    // free edges and vertices
    std::vector< Surface_mesh::Vertex >  free_vertices_;
    std::vector< Surface_mesh::Edge   >  free_edges_;

    // number of example
    unsigned int num_examples_;

    // currently displayed mesh
    int          current_mesh_idx_;

    // is there a basemesh?
    bool base_mesh_set_;

    // is the stiffness set / are edge weights for length and angle computed?
    bool weights_computed_;

    Nonlin_discrete_shell_mode mode_;

};


//=============================================================================
/// @}
//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_NONLINEAR_DISCRETE_SHELL_H
//=============================================================================
