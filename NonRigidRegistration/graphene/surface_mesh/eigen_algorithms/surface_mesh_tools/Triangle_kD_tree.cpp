//== INCLUDES =================================================================

#include "Triangle_kD_tree.h"
#include "diffgeo.h"
#include <graphene/geometry/distance_point_triangle.h>
#include <float.h>


//== NAMESPACES ===============================================================

namespace graphene {
namespace surface_mesh {


//=============================================================================


Triangle_kD_tree::
Triangle_kD_tree(const Surface_mesh&  mesh,
                 unsigned int  max_handles,
                 unsigned int  max_depth)
{
    // init
    root_ = new Node();
    root_->faces_ = new Triangles();
    Surface_mesh::Vertex_property<Point> points = mesh.get_vertex_property<Point>("v:point");


    // collect triangles
    Triangle tri;
    root_->faces_->reserve(mesh.n_faces());
    for (Surface_mesh::Face_iterator fit=mesh.faces_begin(); fit!=mesh.faces_end(); ++fit)
    {
        Surface_mesh::Vertex_around_face_circulator vfit = mesh.vertices(*fit);
        tri.x[0] = points[*vfit];
        ++vfit;
        tri.x[1] = points[*vfit];
        ++vfit;
        tri.x[2] = points[*vfit];
        tri.f  = *fit;
        root_->faces_->push_back(tri);
    }


    // call recursive helper
    _build(root_, max_handles, max_depth);
    //int depth = _build(root_, max_handles, max_depth);
    //LOG(Log_info) << "kD tree depth: " << max_depth - depth << std::endl;
}


//-----------------------------------------------------------------------------


unsigned int
Triangle_kD_tree::
_build(Node*         node,
       unsigned int  max_handles,
       unsigned int  depth)
{
    // should we stop at this level ?
    if ((depth==0) || (node->faces_->size() <= max_handles))
        return depth;


    std::vector<Triangle>::const_iterator fit, fend=node->faces_->end();
    unsigned int i;

    // compute bounding box
    Point bbmax(-FLT_MAX), bbmin(FLT_MAX), p;
    for (fit=node->faces_->begin(); fit!=fend; ++fit)
    {
        for (i=0; i<3; ++i)
        {
            bbmin.minimize(fit->x[i]);
            bbmax.maximize(fit->x[i]);
        }
    }


    // split longest side of bounding box
    Point   bb = bbmax - bbmin;
    Scalar  length = bb[0];
    int     axis   = 0;
    if (bb[1] > length) length = bb[(axis=1)];
    if (bb[2] > length) length = bb[(axis=2)];


#if 1
    // split in the middle
    Scalar split = 0.5 * (bbmin[axis] + bbmax[axis]);
#else
    // find split position as median
    std::vector<Scalar> v; v.reserve(node->faces_->size()*3);
    for (fit=node->faces_->begin(); fit!=fend; ++fit)
        for (i=0; i<3; ++i)
            v.push_back(fit->x[i][axis]]);
    std::sort(v.begin(), v.end());
    split = v[v.size()/2];
#endif


    // create children
    Node *left = new Node();
    left->faces_ = new Triangles();
    left->faces_->reserve(node->faces_->size()/2);
    Node *right = new Node();
    right->faces_ = new Triangles;
    right->faces_->reserve(node->faces_->size()/2);


    // partition for left and right child
    for (fit=node->faces_->begin(); fit!=fend; ++fit)
    {
        bool l=false, r=false;

        const Triangle& t = *fit;
        if (t.x[0][axis] <= split) l=true; else r=true;
        if (t.x[1][axis] <= split) l=true; else r=true;
        if (t.x[2][axis] <= split) l=true; else r=true;

        if (l)
        {
            left->faces_->push_back(t);
        }

        if (r)
        {
            right->faces_->push_back(t);
        }
    }


    // stop here?
    if (left->faces_->size()  == node->faces_->size() ||
        right->faces_->size() == node->faces_->size())
    {
        // compact my memory
        std::vector<Triangle>(*node->faces_).swap(*node->faces_);

        // delete new nodes
        delete left;
        delete right;

        // return tree depth
        return depth;
    }

    // or recurse further?
    else
    {
        // free my memory
        delete node->faces_;
        node->faces_ = 0;

        // store internal data
        node->axis_  = axis;
        node->split_ = split;
        node->left_child_  = left;
        node->right_child_ = right;

        // recurse to childen
        int depth_left  = _build(node->left_child_,  max_handles, depth-1);
        int depth_right = _build(node->right_child_, max_handles, depth-1);

        return std::min(depth_left, depth_right);
    }
}


//-----------------------------------------------------------------------------


Triangle_kD_tree::Nearest_neighbor
Triangle_kD_tree::nearest(const Point& p) const
{
    Nearest_neighbor data;
    data.dist = FLT_MAX;
    data.tests = 0;
    _nearest(root_, p, data);
    //std::cerr << "tests: " << data.tests << std::endl;
    return data;
}


//-----------------------------------------------------------------------------


void
Triangle_kD_tree::
_nearest(Node* node, const Point& point, Nearest_neighbor& data) const
{
    // terminal node?
    if (!node->left_child_)
    {
        Scalar d;
        Point  n;

        std::vector<Triangle>::const_iterator fit=node->faces_->begin(), fend=node->faces_->end();
        for (; fit!=fend; ++fit)
        {
            d  = geometry::dist_point_triangle(point, fit->x[0], fit->x[1], fit->x[2], n);
            ++data.tests;
            if (d < data.dist)
            {
                data.dist    = d;
                data.face    = fit->f;
                data.nearest = n;
            }
        }
    }



    // non-terminal node
    else
    {
        Scalar dist = point[node->axis_] - node->split_;

        if (dist <= 0.0)
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
