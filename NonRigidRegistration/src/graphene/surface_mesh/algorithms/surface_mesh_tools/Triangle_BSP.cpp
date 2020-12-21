//== INCLUDES =================================================================

#include "Triangle_BSP.h"
#include <graphene/geometry/distance_point_triangle.h>
#include <float.h>


//== NAMESPACES ===============================================================

namespace graphene {
namespace surface_mesh {


//=============================================================================


Triangle_BSP::Node::Node(Node* parent)
: parent_(parent), left_child_(0), right_child_(0)
{}


//-----------------------------------------------------------------------------


Triangle_BSP::Node::
~Node()
{
    delete left_child_;
    delete right_child_;

    if (parent_)
    {
        if (this == parent_->left_child_)
            parent_->left_child_ = 0;
        else
            parent_->right_child_ = 0;
    }
}


//-----------------------------------------------------------------------------


Triangle_BSP::
Triangle_BSP(Surface_mesh&    mesh)
: mesh_(mesh), root_(0)
{
    points_ = mesh_.get_vertex_property<Point>("v:point");
}


//-----------------------------------------------------------------------------


unsigned int
Triangle_BSP::
build(unsigned int  max_handles, unsigned int  max_depth)
{
    // init
    root_ = new Node(0);

    // collect triangles
    root_->faces_.reserve(mesh_.n_faces());
    for (Surface_mesh::Face_iterator fit=mesh_.faces_begin(); fit!=mesh_.faces_end(); ++fit)
        root_->faces_.push_back(*fit);

    // call recursive helper
    int depth = _build(root_, max_handles, max_depth);

    std::cerr << "depth = " << max_depth - depth << std::endl;
    return max_depth - depth;
}


//-----------------------------------------------------------------------------


unsigned int
Triangle_BSP::
_build(Node*         node,
       unsigned int  max_handles,
       unsigned int  depth)
{
    // should we stop at this level ?
    if ((depth==0) || (node->faces_.size() <= max_handles))
        return depth;


    std::vector<Surface_mesh::Face>::const_iterator fit, fend=node->faces_.end();
    Surface_mesh::Vertex_around_face_circulator vfit;


    // compute bounding box
    Point bbmax(-FLT_MAX), bbmin(FLT_MAX), p;
    for (fit=node->faces_.begin(); fit!=fend; ++fit)
    {
        vfit=mesh_.vertices(*fit);
        p = points_[*vfit];
        bbmin.minimize(p);
        bbmax.maximize(p);
        ++vfit;
        p = points_[*vfit];
        bbmin.minimize(p);
        bbmax.maximize(p);
        ++vfit;
        p = points_[*vfit];
        bbmin.minimize(p);
        bbmax.maximize(p);
    }


    // split longest side of bounding box
    Point   bb = bbmax - bbmin;
    Scalar  length = bb[0];
    int     axis   = 0;
    if (bb[1] > length) length = bb[(axis=1)];
    if (bb[2] > length) length = bb[(axis=2)];


    // construct splitting plane
    const Point XYZ[3] = { Point(1,0,0), Point(0,1,0), Point(0,0,1) };
    node->plane_ = Plane((bbmin+bbmax)*(Scalar)0.5, XYZ[axis]);


    // create children
    node->left_child_  = new Node(node);
    node->right_child_ = new Node(node);
    node->left_child_->faces_.reserve(node->faces_.size()/2);
    node->right_child_->faces_.reserve(node->faces_.size()/2);


    // partition for left and right child
    bool left, right;
    for (fit=node->faces_.begin(); fit!=fend; ++fit)
    {
        left = right = false;

        p = points_[*(vfit=mesh_.vertices(*fit))];
        if (node->plane_(p))  left  = true;
        else                  right = true;

        p = points_[*(++vfit)];
        if (node->plane_(p))  left  = true;
        else                  right = true;

        p = points_[*(++vfit)];
        if (node->plane_(p))  left  = true;
        else                  right = true;

        if (left)  node->left_child_->faces_.push_back(*fit);
        if (right) node->right_child_->faces_.push_back(*fit);
    }


    // stop here?
    if (node->left_child_->faces_.size()  == node->faces_.size() ||
        node->right_child_->faces_.size() == node->faces_.size())
    {
        delete node->left_child_;   node->left_child_  = 0;
        delete node->right_child_;  node->right_child_ = 0;
        return depth;
    }


    // free memory
    std::vector<Surface_mesh::Face>().swap(node->faces_);


    // recurse to childen
    int depth_left  = _build(node->left_child_,  max_handles, depth-1);
    int depth_right = _build(node->right_child_, max_handles, depth-1);
    return std::min(depth_left, depth_right);
}


//-----------------------------------------------------------------------------


Triangle_BSP::Nearest_neighbor
Triangle_BSP::nearest(const Point& p) const
{
    assert(root_);

    Nearest_neighbor data;
    data.dist = FLT_MAX;
    data.tests = 0;
    _nearest(root_, p, data);
    //std::cerr << "tests: " << data.tests << std::endl;
    return data;
}


//-----------------------------------------------------------------------------


void
Triangle_BSP::
_nearest(Node* node, const Point& point, Nearest_neighbor& data) const
{
    // terminal node?
    if (!node->left_child_)
    {
        Scalar d;
        Point  p0, p1, p2, n;
        Surface_mesh::Vertex_around_face_circulator vfit;

        std::vector<Surface_mesh::Face>::const_iterator fit=node->faces_.begin(), fend=node->faces_.end();
        for (; fit!=fend; ++fit)
        {
            vfit=mesh_.vertices(*fit);
            p0 = points_[*vfit];
            p1 = points_[*(++vfit)];
            p2 = points_[*(++vfit)];
            d  = geometry::dist_point_triangle(point, p0, p1, p2, n);

            ++data.tests;

            if (d < data.dist)
            {
                data.dist = d;
                data.face = *fit;
                data.nearest = n;
            }
        }
    }



    // non-terminal node
    else
    {
        Scalar dist = node->plane_.distance(point);

        if (dist > 0.0)
        {
            _nearest(node->left_child_, point, data);
            if (fabs(dist) < data.dist)
                _nearest(node->right_child_, point, data);
        }
        else
        {
            _nearest(node->right_child_, point, data);
            if (fabs(dist) < data.dist)
                _nearest(node->left_child_, point, data);
        }
    }
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
