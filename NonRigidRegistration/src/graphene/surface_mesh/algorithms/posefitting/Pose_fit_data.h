//=============================================================================
#ifndef GRAPHENE_POSE_FIT_H
#define GRAPHENE_POSE_FIT_H
//=============================================================================

//== INCLUDES =================================================================

#include <graphene/geometry/Matrix3x3.h>

#include <graphene/character/data_structure/Character.h>
#include <graphene/geometry/Point_set.h>


#include <map>

//== NAMESPACE ================================================================


namespace graphene {
namespace surface_mesh {

using graphene::character::Character;
using graphene::character::Skeleton;
using graphene::geometry::Point_set;

//== CLASS DEFINITION =========================================================

class Pose_fit_data
{


public:
    Pose_fit_data();
    ~Pose_fit_data();

    //getter/setter
    void set_character(character::Character* character);
    void set_point_set(geometry::Point_set* pointset);

    //references
    std::vector<std::string>& free_joints() { return free_joints_; }
    std::map<std::string, std::pair<Vec3f, Vec3f> >& joint_limits() { return joint_limits_; }

    void correct_joint_positions();

    void inverse_pose();

    void compute_mvc_of_joints();

    void compute_local_vrot();

    bool ready();

    bool save_dof(const std::string& filename);
    bool load_dof(const std::string& filename);

    Character* character() { return character_; }
    Surface_mesh* mesh()   { return mesh_; }
    Point_set* point_set() { return point_set_; }

private:

    Character* character_;
    Surface_mesh* mesh_;

    Point_set* point_set_;


    std::vector<std::string> free_joints_;

    std::map<std::string, std::pair<Vec3f, Vec3f> > joint_limits_;
};

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_POSE_FIT_H
//=============================================================================
