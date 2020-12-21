#ifndef POINTSET_PROCESSING_FUNCTIONS_H
#define POINTSET_PROCESSING_FUNCTIONS_H

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/scene_graph/Point_set_node.h>

using namespace graphene;

namespace graphene
{


class Pointset_processing
{
private:
    scene_graph::Point_set_node& pointset_node_;

    std::vector<Vec3f> lm_backup_points_;
    std::vector<Vec3f> lm_backup_normals_;
    std::vector<Vec3f> lm_backup_colors_;
    std::vector<Vec3f> lm_backup_ear_points_;
    std::vector<Vec3f> lm_backup_ear_normals_;
    std::vector<Vec3f> lm_backup_ear_colors_;
public:
    Pointset_processing(scene_graph::Point_set_node& pointset_node) :
        pointset_node_(pointset_node)
    {}


void subsampling_according_to_mesh_meanedge(const surface_mesh::Surface_mesh& mesh, double meanedge_factor);

void replace_biggest_plane_ransac(float ymin, float ymax);

private:

void backup_landmarks(geometry::Point_set& point_set);

void restore_landmarks(geometry::Point_set& point_set);

};



}



#endif
