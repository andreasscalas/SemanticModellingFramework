//=============================================================================

#ifndef TRIANGLE_KDTREE_H
#define TRIANGLE_KDTREE_H


//== INCLUDES =================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <vector>


//== NAMESPACES ===============================================================

namespace graphene {
namespace surface_mesh {


/// \addtogroup surface_mesh
/// @{

//== CLASS DEFINITION =========================================================



class Triangle_kD_tree
{
public:

    /// construct with mesh
    Triangle_kD_tree(const Surface_mesh& mesh,
                     unsigned int  max_faces=10,
                     unsigned int  max_depth=30);

    /// destructur
    ~Triangle_kD_tree() { delete root_; }

    /// nearest neighbor information
    struct Nearest_neighbor
    {
        Scalar              dist;
        Surface_mesh::Face  face;
        Point               nearest;
        int                 tests;
    };


    /// Return handle of the nearest neighbor
    Nearest_neighbor nearest(const Point& p) const;



private:

    // triangle stores corners and face handle
    struct Triangle
    {
        Triangle() {}
        Triangle(const Point& x0, const Point& x1, const Point& x2, Surface_mesh::Face ff)
        { x[0]=x0; x[1]=x1; x[2]=x2; f=ff; }

        Point x[3];
        Surface_mesh::Face f;
    };


    // vector of Triangle
    typedef std::vector<Triangle>  Triangles;


    // Node of the tree: contains parent, children and splitting plane
    struct Node
    {
        Node() : faces_(0), left_child_(0), right_child_(0) {}

        ~Node()
        {
            delete faces_;
            delete left_child_;
            delete right_child_;
        }

        unsigned char axis_;
        Scalar        split_;
        Triangles     *faces_;
        Node          *left_child_;
        Node          *right_child_;
    };


    // Recursive part of build()
    unsigned int _build(Node*        node,
                        unsigned int max_handles,
                        unsigned int depth);

    // Recursive part of nearest()
    void _nearest(Node* node, const Point& point, Nearest_neighbor& data) const;



private:

    Node *root_;
};

/// @}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
