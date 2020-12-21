//=============================================================================
//
//  CLASS Point_BSP_tree
//
//=============================================================================

#ifndef POINT_BSP_TREE_H
#define POINT_BSP_TREE_H


//== INCLUDES =================================================================


#include <graphene/types.h>
#include <vector>
#include <graphene/utility/Priority_queue.h>


namespace graphene {
namespace geometry {


//== CLASS DEFINITION =========================================================

/// \addtogroup geometry
/// @{

/// A class representing a BSP tree for points.
class Point_BSP_tree
{
public:

    //-------------------------------------------------------------- public types

    typedef std::vector<Point>      Points;
    typedef Points::const_iterator  ConstPointIter;


    // the elemnent stored in the tree
    struct Element
    {
        Element(const Point& _p, int _idx) : point(_p), idx(_idx) {}
        Point point;
        int   idx;
    };

    typedef std::vector<Element>      Elements;
    typedef Elements::iterator        ElementIter;
    typedef Elements::const_iterator  ConstElementIter;


    // Functor for partitioning wrt splitting plane
    struct PartPlane
    {
        PartPlane(unsigned char _cut_dim, Scalar _cut_val)
            : cut_dim_(_cut_dim), cut_val_(_cut_val) {}

        bool operator()(const Element& _e) const { return _e.point[cut_dim_] > cut_val_; }

        unsigned char   cut_dim_;
        Scalar          cut_val_;
    };


    // priority queue for kNN
    typedef ::graphene::utility::Max_priority_queue<int,float> PrioQ;


    // Store nearest neighbor information
    struct NearestNeighborData
    {
        Point          ref, off;
        Scalar         dist;
        int            nearest;
        unsigned int   leaf_tests;
    };


    // Store kNN information
    struct kNearestNeighborData
    {
        Point          ref, off;
        Scalar         dist;
        PrioQ          k_nearest;
        unsigned int   leaf_tests;
    };


    // Store ball information
    struct ballData
    {
        Point          ref, off;
        Scalar         dist;
        unsigned int   leaf_tests;
    };


    // Node of the tree: contains parent, children and splitting plane
    struct Node
    {
        Node(ElementIter _begin, ElementIter _end)
            : left_child_(0), right_child_(0), begin_(_begin), end_(_end) {}

        ~Node()
        {
            delete left_child_;
            delete right_child_;
        }

        Node *left_child_, *right_child_;
        ElementIter begin_, end_;
        unsigned char  cut_dim_;
        Scalar cut_val_;
    };



public:

    //----------------------------------------------------------- public methods

    /** Constructor: need traits that define the types and
        give us the points by traits_.point(PointHandle) */
    Point_BSP_tree(const Points& _points) : points_(_points), root_(0) {}

    /// Destructor
    ~Point_BSP_tree() { delete root_; }

    /// Build the tree. Returns number of nodes.
    unsigned int build(unsigned int _max_handles, unsigned int _max_depth);

    /// Return handle of the nearest neighbor
    NearestNeighborData nearest(const Point& _p) const;

    /// Return handles of the k nearest neighbors by inserting them
    /// into an container using _inserter (e.g. std::back_inserter)
    int k_nearest(const Point& _p, unsigned int _k, std::vector<int>& _knn) const;

    /// Return handles of the neighbors within a ball centered at p by
    /// inserting them into a container
    int ball(const Point& p, Scalar radius, std::vector<int>& ball) const;


private:

    //----------------------------------------------------------- private methods

    // Recursive part of build()
    void _build(Node*        _node,
                unsigned int _max_handles,
                unsigned int _depth);

    // Recursive part of nearest()
    void _nearest(Node* _node, Scalar rd, NearestNeighborData& _data) const;


    // Recursive part of k_nearest()
    void _k_nearest(Node* _node, Scalar _rd, kNearestNeighborData& _data) const;


    // Recursive part of ball()
    void _ball(Node* _node, Scalar _rd, ballData& _data, Scalar radius_sq, std::vector<int>& ball) const;


    //-------------------------------------------------------------- private data

    const Points&  points_;
    Elements       elements_;
    Node           *root_;
    unsigned int   n_nodes_;
};

/// @}

//=============================================================================
} // geometry
} // graphene
//=============================================================================
#endif // POINT_BSP_TREE_H defined
//=============================================================================
