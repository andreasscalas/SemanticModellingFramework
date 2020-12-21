//=============================================================================
// Copyright (C) 2013 by Graphics & Geometry Group, Bielefeld University
//=============================================================================


#ifndef GRAPHENE_GEOMETRY_REPRESENTATION_H
#define GRAPHENE_GEOMETRY_REPRESENTATION_H


//== INCLUDES =================================================================


#include <graphene/types.h>

#include <vector>


//== NAMESPACES ===============================================================


namespace graphene {
namespace geometry {


//== CLASS DEFINITION =========================================================

class Geometry_representation
{

public: //---------------------------------------------------- public functions


    virtual std::vector<Point>& points() = 0;


protected: //---------------------------------------------- protected functions


protected: //--------------------------------------------------- protected data

};


//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_GEOMETRY_REPRESENTATION_H
//=============================================================================
