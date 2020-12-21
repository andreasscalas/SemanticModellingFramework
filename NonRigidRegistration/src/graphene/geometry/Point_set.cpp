//=============================================================================
// Copyright (C) 2013 by Graphics & Geometry Group, Bielefeld University
//=============================================================================


//== INCLUDES =================================================================


#include <graphene/geometry/Point_set.h>
#include <fstream>


//== NAMESPACES ===============================================================


namespace graphene {
namespace geometry {


//=============================================================================

Point_set::~Point_set()
{
    if (! autosave_landmarks_filename_.empty() && !landmarks_.empty())
    {

        std::string ext;
        ext = autosave_landmarks_filename_.substr(autosave_landmarks_filename_.rfind('.') + 1);
        if (ext == "sel")
        {
            std::ofstream ofs;
            ofs.open(autosave_landmarks_filename_.c_str());

            if (!ofs)
                return;


            for (size_t i=0; i < landmarks_.size();++i)
            {
                ofs << landmarks_[i] << "\n";
            }
        }
        else
        {
            std::cerr << "Point_set::~Point_set: [WARNING] Cannot write landmarks to \"" << autosave_landmarks_filename_ << "\"" << std::endl;
            return;
        }
    }
}

//-----------------------------------------------------------------------------


void
Point_set::
clear()
{
    points_.clear();
    normals_.clear();
    colors_.clear();

    landmarks_.clear();
    lm_ears_.clear();
    selected_indices_.clear();

    temp_transformation_ = Mat4f::identity();
}


//-----------------------------------------------------------------------------


bool
Point_set::
read(const char* filename)
{
    std::ifstream ifs(filename);

    if (! ifs.good())
        return false;

    Point p;
    Normal n;
    Color c;
    int num_elements=0;

    points_.clear();
    normals_.clear();


    std::string line;
    std::stringstream ss;

    //get first line to determine format: our/standard or agisoft's
    std::getline(ifs, line);
    ss.str(line);
    while (true)
    {
        ss >> p[0];
        if (!ss)
            break;
        ++num_elements;
    }

    ifs.seekg(0);
    if (num_elements == 9)
    {
        while (true)
        {
            std::getline(ifs, line);
            if (!ifs)
                break;
            ss.clear();
            ss.str(line);
            ss >> p[0] >> p[1] >> p[2];
            ss >> c[0] >> c[1] >> c[2];
            ss >> n[0] >> n[1] >> n[2];
            n.normalize();
            points_.push_back(p);
            colors_.push_back(c/255.0f);
            normals_.push_back(n);
        }
    }
    else
    {
        while (true)
        {
            ifs >> p[0] >> p[1] >> p[2];
            ifs >> n[0] >> n[1] >> n[2];
            if (!ifs)
                break;
            points_.push_back(p);
            n.normalize();
            normals_.push_back(n);
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Point_set::
read_cxyz(const char* filename)
{
    std::ifstream ifs(filename);

    if (! ifs.good())
        return false;

    float x, y, z;
    float nx, ny, nz;
    float cx, cy, cz;

    points_.clear();
    normals_.clear();
    colors_.clear();

    while (!ifs.eof())
    {
        ifs >> x >> y >> z;
        points_.push_back(Point(x,y,z));

        ifs >> nx >> ny >> nz;
        Normal the_normal(nx,ny,nz);
        the_normal.normalize();
        normals_.push_back(the_normal);

        ifs >> cx >> cy >> cz;
        cx /= 255.0;
        cy /= 255.0;
        cz /= 255.0;
        colors_.push_back(Color(cx,cy,cz));
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Point_set::
read_photoscan_txt(const char* filename)
{
    std::ifstream ifs(filename);

    if (! ifs.good())
        return false;

    float x, y, z;
    float cx, cy, cz;
    float nx, ny, nz;

    points_.clear();
    normals_.clear();
    colors_.clear();

    while (!ifs.eof())
    {
        ifs >> x >> y >> z;
        ifs >> cx >> cy >> cz;
        ifs >> nx >> ny >> nz;

        if (!ifs)
            break;

        points_.push_back(Point(x,y,z));
        cx /= 255.0;
        cy /= 255.0;
        cz /= 255.0;
        colors_.push_back(Color(cx,cy,cz));
        Normal the_normal(nx,ny,nz);
        the_normal.normalize();
        normals_.push_back(the_normal);
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Point_set::
write(const char* filename) const
{
    std::ofstream ofs(filename);

    if (! ofs.good())
        return false;

    for (unsigned int i(0); i < points_.size(); i++)
    {
        ofs << points_[i];
        ofs << " ";
        ofs << normals_[i];
        ofs << std::endl;
    }

    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
Point_set::
write_cxyz(const char* filename) const
{
    std::ofstream ofs(filename);

    if (! ofs.good())
        return false;

    for (unsigned int i(0); i < points_.size(); i++)
    {
        ofs << points_[i];
        ofs << " ";
        ofs << normals_[i];
        ofs << " ";
        ofs << colors_[i][0] << " " << colors_[i][1] << " " << colors_[i][2];
        ofs << std::endl;
    }

    ofs.close();

    return true;
}

//-----------------------------------------------------------------------------

bool
Point_set::
write_photoscan_txt(const char *filename) const
{
    std::ofstream ofs(filename);

    if (! ofs.good())
        return false;

    if (points_.size() != colors_.size() || points_.size() != normals_.size())
        return false;

    for (unsigned int i(0); i < points_.size(); i++)
    {
        ofs << points_[i];
        ofs << " ";
        ofs << (unsigned int)(colors_[i][0]*255) << " " << (unsigned int)(colors_[i][1]*255) << " " << (unsigned int)(colors_[i][2]*255);
        ofs << " ";
        ofs << normals_[i];
        ofs << "\n";
    }

    ofs.close();

    return true;
}

//=============================================================================
} // namespace geometry
} // namespace graphene
//=============================================================================
