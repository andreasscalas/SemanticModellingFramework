//=============================================================================
//
//  CLASS Point_BSP_tree
//
//=============================================================================


//== INCLUDES =================================================================


#include "Point_BSP_tree.h"
#include <algorithm>
#include <float.h>
#include <graphene/utility/Priority_queue.h>


//=============================================================================


namespace graphene {
namespace geometry {


//== CLASS DEFINITION =========================================================


unsigned int
Point_BSP_tree::build(unsigned int _max_handles, unsigned int _max_depth)
{
    // copy points to element array
    elements_.clear();
    elements_.reserve(points_.size());
    ConstPointIter p_it;
    int i;

    for (p_it=points_.begin(), i=0; p_it!=points_.end(); ++p_it, ++i)
        elements_.push_back( Element(*p_it, i) );


    // init
    delete root_;
    root_ = new Node(elements_.begin(), elements_.end());
    n_nodes_ = 0;


    // call recursive helper
    _build(root_, _max_handles, _max_depth);


    return n_nodes_;
}


//-----------------------------------------------------------------------------


void
Point_BSP_tree::
_build(Node*         _node,
       unsigned int  _max_handles,
       unsigned int  _depth)
{
    /// \todo Recursively build up the BSP tree.
    /// \li Check if the maximum \c _depth is reached or the number of handles is
    ///     smaller than \c _max_handles.
    /// \li Compute the bounding box using the element iterators stored in
    ///     _node->begin_ and _node->end_.
    /// \li Compute the cut value by splitting the longest side of the
    ///     bounding box. Store the axis (0,1 or 2) in _node_->cut_dim_ and
    ///     the cut value in _node_->cut_val_.
    /// \li Determine the partitioning element by calling
    ///     std::partition(_node->begin_, _node->end_, PartPlane(axis, cv))
    /// \li Create left and right children (store the pointers in
    ///     _node->left_child_ and _node->right_child_), increase n_nodes_
    ///     accordingly.
    /// \li Recursively call _build() on the left and right subtrees.

#ifndef STRIP_CODE

    const unsigned int n = _node->end_-_node->begin_;


    // should we stop at this level ?
    if ((_depth == 0) || (n < _max_handles))
        return;


    // compute bounding box
    ElementIter it(_node->begin_);
    Point bb_min = it->point;
    Point bb_max = it->point;
    for (; it!=_node->end_; ++it)
    {
        bb_min.minimize( it->point );
        bb_max.maximize( it->point );
    }


    // split longest side of bounding box
    Point bb = bb_max - bb_min;
    Scalar length = bb[0];
    int axis = 0;
    if (bb[1] > length) length = bb[axis=1];
    if (bb[2] > length) length = bb[axis=2];
    Scalar cv = 0.5*(bb_min[axis]+bb_max[axis]);


    // store cut dimension and value
    _node->cut_dim_ = axis;
    _node->cut_val_ = cv;


    // partition for left and right child
    it = std::partition(_node->begin_, _node->end_, PartPlane(axis, cv));


    // create children
    n_nodes_ += 2;
    _node->left_child_  = new Node(_node->begin_, it);
    _node->right_child_ = new Node(it, _node->end_);


    // recurse to childen
    _build(_node->left_child_,  _max_handles, _depth-1);
    _build(_node->right_child_, _max_handles, _depth-1);

#endif // STRIP_CODE
}


//-----------------------------------------------------------------------------


Point_BSP_tree::NearestNeighborData
Point_BSP_tree::
nearest(const Point& _p) const
{
    // init data
    NearestNeighborData  data;
    data.ref        = _p;
    data.off        = Point(0,0,0);
    data.dist       = FLT_MAX;
    data.leaf_tests = 0;

    // recursive search
    _nearest(root_, 0.0, data);

    // dist was computed as sqr-dist
    data.dist = sqrt(data.dist);

    return data;
}


//-----------------------------------------------------------------------------


void
Point_BSP_tree::
_nearest(Node* _node, Scalar rd, NearestNeighborData& _data) const
{
    if (_node->left_child_)
    {
        Scalar old_off = _data.off[_node->cut_dim_];
        Scalar new_off = _data.ref[_node->cut_dim_] - _node->cut_val_;

        if (new_off > 0.0)
        {
            _nearest(_node->left_child_, rd, _data);
            rd -= old_off*old_off;  rd += new_off*new_off;
            if (rd < _data.dist)
            {
                _data.off[_node->cut_dim_] = new_off;
                _nearest(_node->right_child_, rd, _data);
                _data.off[_node->cut_dim_] = old_off;
            }
        }
        else
        {
            _nearest(_node->right_child_, rd, _data);
            rd -= old_off*old_off;  rd += new_off*new_off;
            if (rd < _data.dist)
            {
                _data.off[_node->cut_dim_] = new_off;
                _nearest(_node->left_child_, rd, _data);
                _data.off[_node->cut_dim_] = old_off;
            }
        }
    }

    // terminal node
    else
    {
        ++_data.leaf_tests;
        Scalar dist;

        for (ElementIter it=_node->begin_; it!=_node->end_; ++it)
        {
            dist = sqrnorm(it->point - _data.ref);
            if (dist < _data.dist)
            {
                _data.dist    = dist;
                _data.nearest = it->idx;
            }
        }
    }
}


//-----------------------------------------------------------------------------


int
Point_BSP_tree::
k_nearest(const Point& _p, unsigned int _k, std::vector<int>& _knn) const
{
    kNearestNeighborData  data;
    data.ref  = _p;
    data.off  = Point(0,0,0);
    data.dist = FLT_MAX;
    data.k_nearest.setSize(_k);
    data.k_nearest.insert(-1, FLT_MAX);
    data.leaf_tests = 0;

    _k_nearest(root_, 0.0, data);

    int k = data.k_nearest.getNofElements();
    _knn.resize(k);
    for (int i=k-1; i>=0; --i)
    {
        _knn[i] = data.k_nearest.getMaxIndex();
        data.k_nearest.removeMax();
    }

    return data.leaf_tests;
}


//-----------------------------------------------------------------------------


void
Point_BSP_tree::
_k_nearest(Node* _node, Scalar rd, kNearestNeighborData& _data) const
{
    // non-terminal node
    if (_node->left_child_)
    {
        int cd = _node->cut_dim_;
        Scalar old_off = _data.off[cd];
        Scalar new_off = _data.ref[cd] - _node->cut_val_;

        if (new_off > 0.0)
        {
            _k_nearest(_node->left_child_, rd, _data);
            rd += -old_off*old_off + new_off*new_off;
            if (rd < _data.dist)
            {
                _data.off[cd] = new_off;
                _k_nearest(_node->right_child_, rd, _data);
                _data.off[cd] = old_off;
            }
        }
        else
        {
            _k_nearest(_node->right_child_, rd, _data);
            rd += -old_off*old_off + new_off*new_off;
            if (rd < _data.dist)
            {
                _data.off[cd] = new_off;
                _k_nearest(_node->left_child_, rd, _data);
                _data.off[cd] = old_off;
            }
        }
    }


    // terminal node
    else
    {
        ++_data.leaf_tests;
        Scalar dist;

        for (ConstElementIter it=_node->begin_; it!=_node->end_; ++it)
        {
            dist = sqrnorm(it->point - _data.ref);
            if (dist < _data.dist)
            {
                _data.k_nearest.insert(it->idx, dist);
                _data.dist = _data.k_nearest.getMaxWeight();
            }
        }
    }
}


//-----------------------------------------------------------------------------


int
Point_BSP_tree::
ball(const Point& p, Scalar radius, std::vector<int>& ball) const
{
    ball.clear();
    Scalar radius_sq_ = radius*radius;

    ballData  data;
    data.ref  = p;
    data.off  = Point(0,0,0);
    data.dist = FLT_MAX;
    data.leaf_tests = 0;

    _ball(root_, 0.0, data, radius_sq_, ball);

    return data.leaf_tests;
}


//-----------------------------------------------------------------------------


void
Point_BSP_tree::
_ball(Node* _node, Scalar rd, ballData& _data, Scalar radius_sq, std::vector<int>& ball) const
{
    // non-terminal node
    if (_node->left_child_)
    {
        int cd = _node->cut_dim_;
        Scalar old_off = _data.off[cd];
        Scalar new_off = _data.ref[cd] - _node->cut_val_;

        if (new_off > 0.0)
        {
            _ball(_node->left_child_, rd, _data, radius_sq, ball);
            rd += -old_off*old_off + new_off*new_off;
            if (rd < radius_sq)
            {
                _data.off[cd] = new_off;
                _ball(_node->right_child_, rd, _data, radius_sq, ball);
                _data.off[cd] = old_off;
            }
        }
        else
        {
            _ball(_node->right_child_, rd, _data, radius_sq, ball);
            rd += -old_off*old_off + new_off*new_off;
            if (rd < radius_sq)
            {
                _data.off[cd] = new_off;
                _ball(_node->left_child_, rd, _data, radius_sq, ball);
                _data.off[cd] = old_off;
            }
        }
    }


    // terminal node
    else
    {
        ++_data.leaf_tests;
        Scalar dist;

        for (ConstElementIter it=_node->begin_; it!=_node->end_; ++it)
        {
            dist = sqrnorm(it->point - _data.ref);
            if (dist < radius_sq)
            {
                ball.push_back(it->idx);
            }
        }
    }
}


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
