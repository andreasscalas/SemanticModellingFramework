
//== INCLUDES ===================================================================


#include "Pose_fit_data.h"

#include <graphene/geometry/Matrix3x3.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/meanvalue_coords.h>
#include <graphene/surface_mesh/algorithms/templatefitting/utility/my_helper.h>

#include <fstream>
#include <sstream>

//== NAMESPACES ================================================================
namespace graphene {
namespace surface_mesh {
//=============================================================================


Pose_fit_data::Pose_fit_data() :
    character_(NULL),
    mesh_(NULL),
    point_set_(NULL)
{

}

//-----------------------------------------------------------------------------

Pose_fit_data::~Pose_fit_data()
{

}

//-----------------------------------------------------------------------------

void Pose_fit_data::set_character(character::Character *character)
{
    character_ = character;
    if (character_ != NULL)
    {
        mesh_ = character_->get_selected_skin();
    }
    else
    {
        mesh_ = NULL;
    }

}

//-----------------------------------------------------------------------------

void Pose_fit_data::set_point_set(geometry::Point_set *pointset)
{
    point_set_ = pointset;
}


//-----------------------------------------------------------------------------

void Pose_fit_data::correct_joint_positions()
{
    if (character_ == NULL || mesh_ == NULL)
        return;

    character_->correct_joint_positions();
}

void Pose_fit_data::inverse_pose()
{
    character_->skeleton().reset_to_initial(true);

    character_->skeleton().update();
    character_->apply_skinning_on_CPU(true);
}

//-----------------------------------------------------------------------------

bool Pose_fit_data::ready()
{

    if (character_ == NULL || mesh_ == NULL)
    {
        std::cout << "Pose_fit: [ERROR] No character/mesh." << std::endl;
        return false;
    }

    if (point_set_ == NULL)
    {
        std::cout << "Pose_fit: [ERROR] No point set." << std::endl;
        return false;
    }

    if (free_joints_.empty())
    {
        std::cout << "Pose_fit: [ERROR] No free joints." << std::endl;
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------

bool Pose_fit_data::save_dof(const std::string &filename)
{
    std::ofstream ofs(filename.c_str());

    if (!ofs)
        return false;

    size_t i;
    std::map<std::string, std::pair<Vec3f, Vec3f> >::const_iterator mit;

    for (i=0; i < free_joints_.size(); ++i)
    {
        const std::string& fj = free_joints_[i];

        ofs << fj;

        mit = joint_limits_.find(fj);
        if (mit != joint_limits_.end())
        {
            const Vec3f &lmin = mit->second.first;
            const Vec3f &lmax = mit->second.second;
            ofs << " "
                << lmin[0] << " " << lmin[1] << " " << lmin[2] << " "
                << lmax[0] << " " << lmax[1] << " " << lmax[2];
        }
        ofs << "\n";
    }

    ofs.close();

    return true;
}

//-----------------------------------------------------------------------------

bool Pose_fit_data::load_dof(const std::string &filename)
{
    std::ifstream ifs(filename.c_str());

    if (!ifs)
    {
        std::cerr << "Pose_fit_data::load_dof: [ERROR] Could not load DoF for IK. Filename: \"" << filename << "\"." << std::endl;
        return false;
    }

    free_joints_.clear();
    joint_limits_.clear();

    std::string line, joint_name;
    std::stringstream ss;
    int i;

    while (ifs)
    {
        std::getline(ifs, line);
        if (!ifs)
            break;

        if (line.find("#") == 0 || line.find("//") == 0)
            continue;

        ss.clear();
        ss.str(line);

        ss >> joint_name;

        free_joints_.push_back(joint_name);

        if (ss.str().size() > joint_name.size())
        {
            std::pair<Vec3f, Vec3f>& p = joint_limits_[joint_name];
            Vec3f &lmin = p.first;
            Vec3f &lmax = p.second;
            for (i=0; i < 3; ++i)
            {
                ss >> lmin[i];
            }

            for (i=0; i < 3; ++i)
            {
                ss >> lmax[i];
            }
            //convert from degree to radians
            lmax *= M_PI / 180.0f;
            lmin *= M_PI / 180.0f;
        }
    }

    ifs.close();

    return true;
}

//-----------------------------------------------------------------------------

void
Pose_fit_data::
compute_mvc_of_joints()
{
    if (character_ == NULL || mesh_ == NULL)
        return;

    Surface_mesh::Mesh_property<std::vector < std::vector < float > > > mvc =
            mesh_->get_mesh_property<std::vector < std::vector < float > > >("m:mvc_joints");

    if (mvc)
    {
        return;
    }

    size_t i;
    character::Skeleton &skeleton = character_->skeleton();

    character::Joint* joint;
    std::vector<Point> joint_positions(skeleton.joints_.size());

    for (i=0;i < joint_positions.size(); ++i)
    {
        joint = skeleton.joints_[i];
        joint_positions[i] = joint->get_global_translation();
    }

    surface_mesh::mvc::compute_mean_value_coords(mesh_, joint_positions, "m:mvc_joints");
}

//-----------------------------------------------------------------------------

void Pose_fit_data::
compute_local_vrot()
{
    if (character_ == NULL || mesh_ == NULL)
        return;

    Surface_mesh::Vertex_property<Mat3f> lvrot = mesh_->vertex_property<Mat3f>("v:local_vrot");

    compute_local_vertex_rotation(mesh_,lvrot.vector());

}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
