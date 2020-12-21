//=============================================================================
// Copyright (C) Graphics & Geometry Processing Group, Bielefeld University
//=============================================================================


#include <graphene/scene_graph/Base_node.h>
#include <algorithm>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


Base_node::
Base_node(const std::string& name)
    : parent_(nullptr),
      name_(name),
      is_target_(false),
      visible_(true)
{
}


//-----------------------------------------------------------------------------


Base_node::
~Base_node()
{
    // delete children
    while (!children_.empty())
    {
        delete children_.front();
    }

    // remove node from parent's list of children
    if (parent_)
    {
        auto it = std::find(parent_->children().begin(),
                            parent_->children().end(),
                            this);
        parent_->children().erase(it);
    }
}


//-----------------------------------------------------------------------------


std::string&
Base_node::
name()
{
    return name_;
}


//-----------------------------------------------------------------------------


std::string
Base_node::
info() const
{
    return std::string("");
}


//-----------------------------------------------------------------------------


const std::string&
Base_node::
fileinfo() const
{
    return fileinfo_;
}


//-----------------------------------------------------------------------------


void
Base_node::
set_fileinfo(const std::string& fileinfo)
{
    fileinfo_ = fileinfo;
}


//-----------------------------------------------------------------------------


geometry::Bounding_box
Base_node::
bbox() const
{
    return bbox_;
}


//=============================================================================
} // namespace scene_graph
} // namespace graphene
//=============================================================================
