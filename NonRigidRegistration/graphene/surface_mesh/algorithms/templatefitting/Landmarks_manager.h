//=============================================================================
#ifndef GRAPHENE_LANDMARKS_MANAGER_H
#define GRAPHENE_LANDMARKS_MANAGER_H
//=============================================================================

//== INCLUDES =================================================================


#include <string>
#include <vector>
#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/geometry/Point_set.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::geometry::Point_set;


//== CLASS DEFINITION =========================================================


class Landmarks_manager
{
public: //---------------------------------------------------- public functions


    // constructor
    Landmarks_manager();

    bool load(Point_set* pointset, const std::string& filename);
    bool load(Surface_mesh* surface_mesh, const std::string& filename);
    bool load(Surface_mesh* surface_mesh,
              const std::string& filename,
              const std::string& filename_mapping_fullbody_head);
    void save(const Point_set* pointset, const std::string& filename);
    void save(const Surface_mesh* surface_mesh, const std::string& filename);


    void clear();


    size_t n_landmarks_template(const Surface_mesh* template_mesh);
    size_t n_landmarks_pointset(const Point_set* point_set) const;

    void get_landmarks_point_set_stdvec(const Point_set* point_set, std::vector<Point>& landm_ps)       const;
    void get_landmarks_template_stdvec(const Surface_mesh* template_mesh, std::vector<Point>& landm_tm) const;

    void get_indices_pointset(const Point_set* point_set, std::vector<unsigned int>& indices) const;
    void get_indices_template(const Surface_mesh* template_mesh, std::vector<Surface_mesh::Vertex>& indices) const;

    bool check_dimension(const Point_set* point_set, const Surface_mesh* template_mesh) const;

private: //--------------------------------------------------- private functions

    bool read(const std::string& filename, std::vector<unsigned int>& indices);
    bool read(const std::string& filename, std::vector<unsigned int>& indices, std::vector<double>& local_weights);

    bool write(const std::string& filename, const std::vector<unsigned int>& indices);
    bool write_sel(const std::string& filename, const std::vector<unsigned int>& indices);


};


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_LANDMARKS_MANAGER_H
//=============================================================================
