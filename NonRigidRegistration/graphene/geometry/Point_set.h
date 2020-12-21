//=============================================================================
// Copyright (C) 2013, 2014 by Graphics & Geometry Group, Bielefeld University
//=============================================================================


#ifndef GRAPHENE_POINT_SET_H
#define GRAPHENE_POINT_SET_H


//== INCLUDES =================================================================


#include <graphene/geometry/Vector.h>
#include <graphene/geometry/Matrix4x4.h>
#include <graphene/types.h>
#include <graphene/geometry/Geometry_representation.h>

#include <vector>


//== NAMESPACES ===============================================================


namespace graphene {
namespace geometry {


//== CLASS DEFINITION =========================================================


/// \addtogroup geometry
/// @{


/// A class representing a point set with normals.
class Point_set : public Geometry_representation
{

public:

    typedef std::vector<Point>::iterator  Point_iterator;
    typedef std::vector<Normal>::iterator Normal_iterator;


public:

    /// Default constructor
    Point_set() : temp_transformation_(Mat4f::identity()) {};

    /// Construct point set from given points and normals
    Point_set(std::vector<Point>  points,
              std::vector<Normal> normals = std::vector<Normal>())
        : points_(points), normals_(normals), temp_transformation_(Mat4f::identity()) {};

    ~Point_set();

    /// clear
    void clear();

    /// Read a point set with normals from a .xyz file.
    bool read(const char* filename);
    /// Read a point set with normals and color from a .cxyz file.
    bool read_cxyz(const char* filename);
    /// Read a point set with normals and color from a photoscan .txt file.
    bool read_photoscan_txt(const char* filename);

    /// Write a point set with normals to a .xyz file.
    bool write(const char* filename) const;
    /// Write a point set with normals and colors to a .cxyz file.
    bool write_cxyz(const char* filename) const;
    /// Write a point set with normals and colors to a .txt file (photoscan compatible).
    bool write_photoscan_txt(const char* filename) const;

    /// Iterator over all points in the set.
    Point_iterator points_begin() { return points_.begin(); };

    /// Past-the-end point iterator.
    Point_iterator points_end() { return points_.end(); };

    /// Iterator over all normals in the set.
    Normal_iterator normals_begin() { return normals_.begin(); };

    /// Past-the-end normal iterator.
    Normal_iterator normals_end() { return normals_.end(); };

    /// Return size of the point set.
    size_t size() const { return points_.size(); };

    std::vector<Point>& points() { return points_; };
    const std::vector<Point>& points() const { return points_; } ;


public:

    std::vector<Point>  points_;
    std::vector<Normal> normals_;
    std::vector<Color>  colors_;

    std::vector<unsigned int> landmarks_;
    std::vector<unsigned int> lm_ears_;
    std::vector<unsigned int> selected_indices_;

    graphene::Mat4f     temp_transformation_;

//helper to autosave landmarks for automated fitting
private:
    std::string autosave_landmarks_filename_;
public:
    void set_autosave_landmarks_filename(const std::string& fname) { autosave_landmarks_filename_ = fname; }
};

/// @}

//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_POINT_SET_H
//=============================================================================
