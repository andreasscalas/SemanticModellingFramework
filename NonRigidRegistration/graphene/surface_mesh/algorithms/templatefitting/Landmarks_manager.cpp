//== INCLUDES ===================================================================


#include "Landmarks_manager.h"

#include "settings.h"

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>

#include <fstream>
#include <limits>
#include <cfloat>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>



//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ============================================================



Landmarks_manager::
Landmarks_manager()
{
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
load(Point_set *pointset, const std::string &filename)
{

    std::vector<unsigned int> indices;
    if (read(filename, indices))
    {
        pointset->landmarks_ = indices;
        return true;
    }

    return false;
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
load(Surface_mesh *template_mesh, const std::string &filename)
{

    std::vector<unsigned int> indices;
    Surface_mesh::Vertex v;

    std::string ext;
    ext = filename.substr(filename.rfind('.') + 1);
    if (ext == "sel")
    {
        if (read(filename, indices))
        {
            Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks
                    = template_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
            std::vector<Surface_mesh::Vertex>& landmark_vec = landmarks[0];

            landmark_vec.resize(indices.size());
            for (size_t i=0; i < indices.size(); ++i)
            {
                v = Surface_mesh::Vertex((int)indices[i]);
                landmark_vec[i] = v;
            }

            return true;
        }
    }
    else if (ext == "lmvw")
    {
        std::vector<double> local_weights;
        if (read(filename, indices, local_weights))
        {
            Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks
                    = template_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
            std::vector<Surface_mesh::Vertex> &landmark_vec = landmarks[0];

            Surface_mesh::Vertex_property< double > landmarks_lw
                    = template_mesh->vertex_property< double >("v:landmark_weight");
            for (auto v : template_mesh->vertices())
            {
                landmarks_lw[v] = 1.0;
            }

            landmark_vec.resize(indices.size());
            for (size_t i=0; i < indices.size(); ++i)
            {
                v = Surface_mesh::Vertex((int)indices[i]);
                landmark_vec[i] = v;

                landmarks_lw[v] = local_weights[i];
            }

            return true;
        }
    }

    return false;
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
load(Surface_mesh *template_mesh,
     const std::string &filename,
     const std::string& filename_mapping)
{
    std::cout << "[INFO] load landmarks on template w.r.t. a mapping" << std::endl;

    // load mapping
    std::vector<int> mapping_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mapping_vec, filename_mapping))
    {
        std::cerr << "[ERROR] While loading mapping." << std::endl;
        return false;
    }

    std::vector<unsigned int> indices;
    Surface_mesh::Vertex v;

    std::string ext;
    ext = filename.substr(filename.rfind('.') + 1);
    if (ext == "sel")
    {
        if (read(filename, indices))
        {
            Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks
                    = template_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
            std::vector<Surface_mesh::Vertex>& landmark_vec = landmarks[0];

            landmark_vec.resize(indices.size());
            for (size_t i = 0; i < indices.size(); ++i)
            {
                const int map = mapping_vec[ indices[i] ];

                if (map == -1)
                {
                    std::cerr << "[ERROR] map == -1" << std::endl;
                    return false;                
                }

                v = Surface_mesh::Vertex(map);
                landmark_vec[i] = v;
            }

            return true;
        }
    }
    else if (ext == "lmvw")
    {
        std::vector<double> local_weights;
        if (read(filename, indices, local_weights))
        {
            Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks
                    = template_mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
            std::vector<Surface_mesh::Vertex>& landmark_vec = landmarks[0];

            Surface_mesh::Vertex_property< double > landmarks_lw
                = template_mesh->vertex_property< double >("v:landmark_weight");
            for (auto v : template_mesh->vertices())
            {
                landmarks_lw[v] = 1.0;
            }

            landmark_vec.resize(indices.size());
            for (size_t i = 0; i < indices.size(); ++i)
            {
                const int map = mapping_vec[ indices[i] ];

                if (map == -1)
                {
                    std::cerr << "[ERROR] map == -1" << std::endl;
                    return false;                
                }

                v = Surface_mesh::Vertex(map);
                landmark_vec[i] = v;

                landmarks_lw[v] = local_weights[i];
            }

            return true;
        }
    }

    return false;
}


//-----------------------------------------------------------------------------


void
Landmarks_manager::
save(const Point_set *pointset, const std::string &filename)
{
    write(filename, pointset->landmarks_);
}


//-----------------------------------------------------------------------------


void
Landmarks_manager::
save(const Surface_mesh *template_mesh, const std::string &filename)
{
    std::vector<unsigned int> indices;
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks
            = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");

    if (landmarks)
    {
        const std::vector<Surface_mesh::Vertex> &landmark_vec = landmarks[0];
        indices.resize(landmark_vec.size());

        for (size_t i=0; i < landmark_vec.size(); ++i)
        {
            const Surface_mesh::Vertex& v = landmark_vec[i];
            indices[i] = (unsigned int)v.idx();
        }

        write(filename, indices);
    }
}


//-----------------------------------------------------------------------------


void
Landmarks_manager::
clear()
{
}



//-----------------------------------------------------------------------------

size_t
Landmarks_manager::
n_landmarks_template(const Surface_mesh *template_mesh)
{
    auto landmarks = template_mesh->get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");

    if (landmarks)
    {
        const std::vector<Surface_mesh::Vertex> &lms = landmarks[0];
        return lms.size();
    }
    else
    {
        return 0;
    }
}


//-----------------------------------------------------------------------------


size_t
Landmarks_manager::
n_landmarks_pointset(const geometry::Point_set *point_set) const
{
    return point_set->landmarks_.size();
}


//-----------------------------------------------------------------------------


void
Landmarks_manager::
get_landmarks_point_set_stdvec(const Point_set* point_set, std::vector<Point> &landm_ps) const
{
//    std::cerr << "in: 'get_landmarks_point_set_stdvec(...)'" << std::endl;

    landm_ps.resize(point_set->landmarks_.size());

    unsigned int i = 0;
    for (auto v : point_set->landmarks_)
    {
        landm_ps[i] = point_set->points_[v];
        ++i;
    }
}



//-----------------------------------------------------------------------------


void
Landmarks_manager::
get_landmarks_template_stdvec(const Surface_mesh* template_mesh, std::vector<Point> &landm_templ) const
{
//    std::cerr << "in: 'get_landmarks_template_stdvec(...)'" << std::endl;

    auto points = template_mesh->get_vertex_property<Point>("v:point");
    auto landmarks = template_mesh->get_mesh_property<std::vector<Surface_mesh::Vertex> >("m:landmarks");

    if (landmarks)
    {
        const std::vector<Surface_mesh::Vertex>& lm_template = landmarks[0];

        landm_templ.resize(lm_template.size());

        unsigned int i = 0;
        for (auto v : lm_template)
        {
            landm_templ[i] = points[v];
            ++i;
        }
    }
}


//-----------------------------------------------------------------------------


void
Landmarks_manager::
get_indices_pointset(const Point_set *point_set, std::vector<unsigned int>& indices) const
{
    indices = point_set->landmarks_;
}


//-----------------------------------------------------------------------------


void
Landmarks_manager::
get_indices_template(const Surface_mesh* template_mesh, std::vector<Surface_mesh::Vertex>& indices) const
{
    indices.clear();

    Surface_mesh::Mesh_property<std::vector<Surface_mesh::Vertex> > landmarks
            = template_mesh->get_mesh_property<std::vector<Surface_mesh::Vertex> >("m:landmarks");

    if (landmarks)
    {
        indices = landmarks[0];
    }
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
check_dimension(const geometry::Point_set *point_set, const Surface_mesh *template_mesh) const
{
    Surface_mesh::Mesh_property<std::vector<Surface_mesh::Vertex> > landmarks
            = template_mesh->get_mesh_property<std::vector<Surface_mesh::Vertex> >("m:landmarks");

    if (landmarks)
    {
        const std::vector<Surface_mesh::Vertex>& tm_landmarks = landmarks[0];
        if (( tm_landmarks.size() == point_set->landmarks_.size() ) && ! tm_landmarks.empty())
        {
            return true;
        }
        else
        {
            std::cerr << "Landmarks_manager::check_dimension: [ERROR] Landmarks dimensions do not fit: "
                      << point_set->landmarks_.size()
                      << " on pointset vs "
                      << tm_landmarks.size()
                      << " on template."
                      << std::endl;
            return false;
        }
    }
    else
    {
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
read(const std::string &filename, std::vector<unsigned int> &indices)
{
    std::string ext;
    ext = filename.substr(filename.rfind('.') + 1);
    if (ext == "sel")
    {
        return load_selection_from_file(indices, filename);
    }
    else
    {
        std::cerr << "Landmarks_manager::read: [WARNING] Cannot read landmarks from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
read(const std::string &filename, std::vector<unsigned int> &indices, std::vector<double>& local_weights)
{
    std::string ext;
    ext = filename.substr(filename.rfind('.') + 1);
    if (ext == "lmvw")
    {
        return load_selection_from_file_with_weights(indices, local_weights, filename);
    }
    else
    {
        std::cerr << "Landmarks_manager::read: [WARNING] Cannot read landmarks with local weights from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
write(const std::string &filename, const std::vector<unsigned int> &indices)
{
    std::string ext;
    ext = filename.substr(filename.rfind('.') + 1);
    if (ext == "sel")
    {
        return write_sel(filename, indices);
    }
    else
    {
        std::cerr << "Landmarks_manager::read: [WARNING] Cannot read landmarks from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
Landmarks_manager::
write_sel(const std::string &filename, const std::vector<unsigned int> &indices)
{
    std::ofstream ofs;
    ofs.open(filename.c_str());

    if (!ofs)
        return false;


    for (size_t i=0; i < indices.size();++i)
    {
        ofs << indices[i] << "\n";
    }

    return true;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
