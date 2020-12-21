//=============================================================================

#ifndef TRIANGLE_BSP_H
#define TRIANGLE_BSP_H


//== INCLUDES =================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/geometry/Plane.h>
#include <vector>


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


/// \addtogroup surface_mesh
/// @{

//== CLASS DEFINITION =========================================================

/// Triangle BSP
class Triangle_BSP
{
public:

    /// construct with mesh
    Triangle_BSP(Surface_mesh&    mesh);

    /// destructur
    ~Triangle_BSP() { delete root_; }

    /// construct BSP, returns depth of the tree
    unsigned int build(unsigned int  max_faces=10, unsigned int  max_depth=100);


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

    typedef graphene::geometry::Plane Plane;    

    // Node of the tree: contains parent, children and splitting plane
    struct Node
    {
        Node(Node* _parent);
        ~Node();
        Node *parent_, *left_child_, *right_child_;
        std::vector<Surface_mesh::Face> faces_;
        Plane plane_;
    };


private:
    
    // Recursive part of build()
    unsigned int _build(Node*        node,
                        unsigned int max_handles,
                        unsigned int depth);

    // Recursive part of nearest()
    void _nearest(Node* node, const Point& point, Nearest_neighbor& data) const;



private:

    Surface_mesh&  mesh_;
    Node*       root_;
    Surface_mesh::Vertex_property<Point> points_;
};

/// @}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif
//=============================================================================
