
//== INCLUDES ===================================================================


#include "IKPose_fit.h"

#include <graphene/geometry/registration.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Energy.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/crease_normals.h>


//== NAMESPACES ================================================================
namespace graphene {
namespace surface_mesh {
//=============================================================================


IKPose_fit::IKPose_fit(Pose_fit_data *data) :
    data_(data),
    ik_skeleton_(NULL),
    ik_solver_(NULL),
    max_iterations_(100),
    conv_value_(0.05),
    max_normal_angle_(50.0),
    max_dist_(0.1),
    lambda_(1.0),
    use_landmarks_(false),
    use_correspondences_(false),
    is_initialized_(false),
    iter_(0),
    error_(DBL_MAX),
    error_prev_(1.0),
    rel_error_(DBL_MAX)
{

}

//-----------------------------------------------------------------------------

IKPose_fit::~IKPose_fit()
{
    clean_up();
}

//-----------------------------------------------------------------------------

bool IKPose_fit::init()
{
    if (!is_initialized_)
    {
        if (!data_->ready())
        {
            std::cerr <<"IKPose_fit::start: [ERROR] Pose fitting not ready!" << std::endl;
            return false;
        }

        if (use_landmarks_ && ! landmarks_manager_.check_dimension(data_->point_set(), data_->mesh()))
        {
            std::cerr <<"IKPose_fit::start: [ERROR] Landmarks dimensions do not fit!" << std::endl;
            return false;
        }

        clean_up();

        ik_skeleton_ = new ik::Skeleton;
        ik_solver_   = new ik::InverseKinematics(lambda_);

        data_->compute_mvc_of_joints();

        clear();

        build_ik_skeleton_recursive(data_->character()->skeleton().root_);
        ik_skeleton_->getRoot()->update();

        //get out joint pointers and save them separatly for faster and convenient access
        character::Joint* joint;
        ik::Joint *ik_joint;
        size_t i;
        for(i = 0; i < data_->character()->skeleton().joints_.size(); ++i) {
            joint = data_->character()->skeleton().joints_[i];
            ik_joint = ik_skeleton_->getJoint(joint->get_name());
            ik_joints_.push_back(ik_joint);
        }

        create_mapping();

        ik_solver_->setRegularization(false);
        ik_solver_->setPrediction(false);
        ik_solver_->setScaling(false);
        ik_solver_->setClamping(true);

        is_initialized_ = true;
    }

    return true;
}

//-----------------------------------------------------------------------------


void IKPose_fit::clean_up()
{
    clear();

    if (ik_skeleton_)
        delete ik_skeleton_;

    if (ik_solver_)
        delete ik_solver_;

    ik_skeleton_ = NULL;
    ik_solver_   = NULL;

    iter_       = 0;
    error_      = DBL_MAX;
    error_prev_ = 1.0;
    rel_error_  = DBL_MAX;

    is_initialized_ = false;
}


//-----------------------------------------------------------------------------

bool IKPose_fit::start()
{
    if (!init())
    {
        return false;
    }

    const bool ok_minimize = minimize();

    clean_up();

    return ok_minimize;
}

//-----------------------------------------------------------------------------

void IKPose_fit::initial_aligment_with_pca(const std::string &filename_pca_mean, const std::string &filename_pca, double corr_distance, double corr_normal, int max_iter)
{

    std::vector<Mat4f> pose_vector;

    surface_mesh::Energy energy;

    character::Character* template_character = data_->character();


    // load pca mean and set as template
    Surface_mesh mesh_pca_mean;
    mesh_pca_mean.read(filename_pca_mean);
    if (0 == mesh_pca_mean.n_vertices())
    {

        std::cerr << "IKPose_fit::initial_aligment_with_pca: [ERROR] PCA mean mesh is empty." << std::endl;
        return;
    }
    //std::cout << "[DEBUG] mesh_pca_mean.n_vertices(): " << mesh_pca_mean.n_vertices() << std::endl;

    //correspondence settings
    surface_mesh::Correspondences_settings constr_settings;
    constr_settings.filter_dist = true;
    constr_settings.dist_until  = corr_distance;
    constr_settings.filter_normals = true;
    constr_settings.normal_until = corr_normal;
    constr_settings.std_dev_filter = false;
    constr_settings.worst_nPerc_filter = false;
    constr_settings.boundary_filter = true;
    constr_settings.symmetry_filter = false;
    constr_settings.w_distance_1r = false;
    constr_settings.w_distance_1ddMax = false;
    constr_settings.w_dot_nn = false;
    constr_settings.w_distance_huber = false;
    constr_settings.constr_dir = surface_mesh::corresp_dir_ps2mesh;//corresp_dir_mesh2ps;//corresp_dir_ps2mesh;


    // enable/disable energy terms

    surface_mesh::Energy_term* pca_tikhonov;
    pca_tikhonov = energy.energy_term_list().get_term(surface_mesh::ET_TIKHONOV_PCA);
    pca_tikhonov->set_multiply_factor(0.01);

    surface_mesh::Energy_term* pca_lm_term;
    pca_lm_term = energy.energy_term_list().get_term(surface_mesh::ET_PCA_LANDMARKS);
    if (true)
    {
        pca_lm_term->set_weight(1.0);
        pca_lm_term->set_multiply_factor(1.0);
        pca_tikhonov->set_weight(0.0002);
    }

    surface_mesh::Energy_term* pca_cpc_term;
    pca_cpc_term = energy.energy_term_list().get_term(surface_mesh::ET_PCA_CPC_PS2MESH);

    if (true)
    {
        pca_cpc_term->set_weight(1.0);
        pca_cpc_term->set_multiply_factor(1.0);
        pca_tikhonov->set_weight(0.0);

        pca_lm_term->set_weight(0.1);
    }



    set_use_correspondences(true);
    set_use_landmarks(true);


    //load pca model from file
    energy.energy_term_data().pca.load_pca_model(filename_pca);

    if (energy.energy_term_data().pca.dim_pca_model_ == 0)
    {
        std::cerr << "IKPose_fit::initial_aligment_with_pca: [ERROR] PCA model has dimension 0" << std::endl;
        return;
    }


    //create new character from pca mean mesh and template skeleton
    character::Character pca_mean_character;
    pca_mean_character.skeleton() = template_character->skeleton();
    character::Surface_mesh_skin* skin = new character::Surface_mesh_skin(*data_->mesh()); //create new surface mesh skin with weights/depends of template character
    pca_mean_character.skins().push_back(skin);
    pca_mean_character.skeleton().reset_to_initial();
    pca_mean_character.skeleton().set_current_pose_as_bindpose();
    pca_mean_character.skeleton().init();

    skin->points() = mesh_pca_mean.points();
    surface_mesh::crease_normals(skin, 0.0f, "h:normal"); // recompute normals for pca mean shape
    pca_mean_character.correct_joint_positions(); // adjust joint positions for pca mean shape


    //get current pose from template character
    surface_mesh::get_pose_vector(template_character->skeleton(), pose_vector);

    //set objects for template fit
    energy.energy_term_data().point_set = data_->point_set();
    energy.energy_term_data().template_mesh = pca_mean_character.get_selected_skin();
    data_->set_character(&pca_mean_character);

    Surface_mesh::Mesh_property< std::vector<Mat4f> > skinning_scanpose_to_tpose =
            pca_mean_character.get_selected_skin()->mesh_property< std::vector<Mat4f> >("m:skinning_scanpose_to_tpose");

    std::vector<Mat4f> &scanpose_to_tpose = skinning_scanpose_to_tpose[0];

    //double error=DBL_MAX, error_prev=0.0, rel_error=DBL_MAX;
    //const double conv_val = 0.9;
    int iter=0;
    do
    {
        //set pose vector aka scan pose
        surface_mesh::set_pose_vector(pose_vector, pca_mean_character.skeleton());
        //apply skinning/transform vertices
        pca_mean_character.apply_skinning_on_CPU();

        //optimize posture and global registration based on correspondences
        start();

        surface_mesh::compute_correspondences(
                    pca_mean_character.get_selected_skin(),
                    energy.energy_term_data().point_set,
                    constr_settings,
                    energy.energy_term_data().correspondences);

        //backup current posture
        surface_mesh::get_pose_vector(pca_mean_character.skeleton(), pose_vector);

        //reset to bindpose here to have skinning matrices which transform vertices from scan pose to t-pose; important for transforming correspondences to t-pose
        pca_mean_character.skeleton().reset_to_bindpose();
        scanpose_to_tpose = pca_mean_character.skeleton().skinning_matrices_;

        //skin mesh back to tpose
        surface_mesh::skin_mesh("m:skinning_scanpose_to_tpose", pca_mean_character.get_selected_skin(), false);

        //fit shape using PCA
        energy.minimize(0.01);
        //template_fit.start(constr_settings, 10, 0.001, 0.000001, true, true);

        energy.energy_term_list().apply_weight_multipliers();

        //as we are in t-pose here, it is safe and best to correct joint positions here
        pca_mean_character.correct_joint_positions();

        ++iter;
    }
    while (/*rel_error > conv_val  &&*/ iter < max_iter);

    //finally go back to scan pose here since we need it for further processing
    surface_mesh::set_pose_vector(pose_vector, pca_mean_character.skeleton());

    pca_mean_character.apply_skinning_on_CPU();



    //copy solution from temporary pca mean character to template character
    template_character->get_selected_skin()->points() = pca_mean_character.get_selected_skin()->points();
    template_character->skeleton() = pca_mean_character.skeleton();

    data_->set_character(template_character);

}

//-----------------------------------------------------------------------------

void IKPose_fit::clear()
{
    if (ik_skeleton_) ik_skeleton_->clearEffectors();
    if (ik_skeleton_) ik_skeleton_->clearJoints();

    ik_joints_.clear();
}

//-----------------------------------------------------------------------------

bool IKPose_fit::minimize()
{
    do
    {
        if (!minimization_step())
            return false;
    }
    while( !is_converged() );

    std::cout << "Pose fitting with ";

    if (use_landmarks_)
        std::cout << "landmarks";
    if (use_correspondences_ && use_landmarks_) std::cout << " and ";
    if (use_correspondences_)
        std::cout << "correspondences";

    std::cout << " done."  << std::endl;

    //transform_points(M);
    //data_->character()->apply_skinning_on_CPU();
    //data_->character()->set_current_pose_as_bindpose();

    return true;
}

//-----------------------------------------------------------------------------

bool IKPose_fit::minimization_step()
{
    if (!is_initialized_ && !init())
    {
        return false;
    }

    correspondence_settings_.filter_dist = true;
    correspondence_settings_.dist_until  = max_dist_;
    correspondence_settings_.filter_normals = true;
    correspondence_settings_.normal_until = max_normal_angle_;
    correspondence_settings_.std_dev_filter = false;
    correspondence_settings_.worst_nPerc_filter = false;
    correspondence_settings_.boundary_filter = true;
    correspondence_settings_.symmetry_filter = false;
    correspondence_settings_.w_distance_1r = false;
    correspondence_settings_.w_distance_1ddMax = false;
    correspondence_settings_.w_dot_nn = false;
    correspondence_settings_.w_distance_huber = false;
    correspondence_settings_.constr_dir = corresp_dir_ps2mesh;//corresp_dir_mesh2ps;//corresp_dir_ps2mesh;

    size_t i;
//    double error=DBL_MAX;
//    error_prev=0.0;
//    rel_error;
//    iter = 0;
    const double conv_val = conv_value_;
    const int max_iter = max_iterations_;

    Surface_mesh::Vertex_around_face_circulator vfc;
    Surface_mesh::Vertex_property<Point> vpoint = data_->mesh()->get_vertex_property<Point>("v:point");

    targets_.clear();
    positions_.clear();
    correspondences_.clear();


    if (use_landmarks_)
    {
        landmarks_manager_.get_landmarks_point_set_stdvec(data_->point_set(), targets_);
        landmarks_manager_.get_landmarks_template_stdvec(data_->mesh(), positions_);
    }


    if (use_correspondences_)
    {
        if (surface_mesh::compute_correspondences(data_->mesh(), data_->point_set(), correspondence_settings_, correspondences_))
        {
            targets_.reserve(targets_.size() + correspondences_.size());
            positions_.reserve(positions_.size() + correspondences_.size());
            for (i=0; i < correspondences_.size(); ++i)
            {
                const Correspondence& c = correspondences_[i];
                targets_.push_back(c.on_ps);
                positions_.push_back(c.on_template);
            }
        }
    }



    if (targets_.empty() || positions_.empty())
    {
        std::cout << "IKPose_fit::minimize: [ERROR] No targets and/or positions." << std::endl;
        return false;
    }

    const Mat4f M = geometry::registration(targets_,
                                           positions_,
                                           graphene::geometry::CONFORMAL_REGISTRATION);

    for (i=0; i < targets_.size(); ++i)
    {
        targets_[i] = affine_transform(M, targets_[i]);
    }

    transform_points(M);


    set_targets_and_positions(targets_, positions_);
    solve();
    copy_pose();
    data_->character()->skeleton().update();
    data_->character()->apply_skinning_on_CPU(true);

    //update position vector with new positions after mesh was skinned
    positions_.clear();
    positions_.reserve(targets_.size());
    if (use_landmarks_)
    {
        landmarks_manager_.get_landmarks_template_stdvec(data_->mesh(), positions_);
    }

    if (use_correspondences_)
    {
        for (i=0; i < correspondences_.size(); ++i)
        {
            const Correspondence& c = correspondences_[i];
            if (c.constr_dir == corresp_dir_mesh2ps)
            {
                positions_.push_back(vpoint[c.on_template_vh]);
            }
            else if (c.constr_dir == corresp_dir_ps2mesh)
            {
                vfc = data_->mesh()->vertices(c.face);

                const Point& p0 = vpoint[*vfc];
                const Point& p1 = vpoint[*(++vfc)];
                const Point& p2 = vpoint[*(++vfc)];

                const Vec3f& bc = c.on_template_bc;
                positions_.push_back(bc[0]*p0 + bc[1]*p1 + bc[2]*p2);
            }
        }
    }


    if (targets_.size() != positions_.size())
    {
        std::cout << "IKPose_fit::minimize: [ERROR] #targets != # positions." << std::endl;
        return false;
    }

    error_prev_ = error_;
    error_ = 0.0;

    for (i=0; i < targets_.size(); ++i)
    {
        //std::cout << corr_ps[i] << " - " << corr_templ[i] << std::endl;
        const double u = norm(targets_[i] - positions_[i]);
        error_ += u*u;
    }
    error_ *= (1.0/targets_.size());

    rel_error_ = ( std::abs(error_ - error_prev_) / error_prev_ );

    ++iter_;

    printf("Pose fitting iteration %d | E = %.6f | rel. E = %.6f/%f\n", iter_, error_, rel_error_, conv_val);

    return true;
}

//-----------------------------------------------------------------------------
/*
void IKPose_fit::minimize_feet()
{
    if (!data_->ready())
        return;

    size_t i;
    int iter=0;
    const double conv_val = conv_value_;
    const int max_iter = 1;//max_iterations_;
    double error=DBL_MAX, error_prev=0.0, rel_error;

    Surface_mesh *mesh = data_->mesh();

    Surface_mesh::Mesh_property<std::vector<Surface_mesh::Vertex> > mgroundvertices = mesh->get_mesh_property<std::vector<Surface_mesh::Vertex> >("m:ground_vertices");
    Surface_mesh::Vertex_property<Point > vpoint = mesh->get_vertex_property<Point>("v:point");


    if (!mgroundvertices)
        return;


    const std::vector<Surface_mesh::Vertex> &gv = mgroundvertices[0];

    targets_.reserve(gv.size());
    positions_.reserve(gv.size());

    //float acc_avg_dist=0.0f;
    float avg_dist=0.0f;

    character::Skeleton &skeleton = data_->character()->skeleton();

    const Vec3f p_before  = skeleton.root_->get_global_translation();

    do
    {

        avg_dist = 0.0f;
        for (i=0; i < gv.size(); ++i)
        {
            const Point &p = vpoint[gv[i]];

            avg_dist += ground_level_ - p[1];
        }
        avg_dist *= 1.0f/gv.size();

        ik_skeleton_->getRoot()->translate(ik::Vec3f(0.0f,avg_dist,0.0f));
        ik_skeleton_->setInitialTransformation(ik_skeleton_->getRoot()->getName(), ik_skeleton_->getRoot()->getLocalTransformation());

        targets_.clear();
        positions_.clear();

        for (i=0; i < gv.size(); ++i)
        {
            const Vec3f& p = vpoint[gv[i]];
            positions_.push_back(p + Vec3f(0.0f,avg_dist,0.0f));
            targets_.push_back(Vec3f(p[0], ground_level_, p[2]));
        }

        set_targets_and_positions(targets_, positions_);
        solve();
        copy_pose();
        data_->character()->skeleton().update();
        data_->character()->apply_skinning_on_CPU(true);

        targets_.clear();
        positions_.clear();

        for (i=0; i < gv.size(); ++i)
        {
            const Vec3f& p = vpoint[gv[i]];
            positions_.push_back(p);
            targets_.push_back(Vec3f(p[0], ground_level_, p[2]));
        }

        error_prev = error;
        error = 0.0;

        for (i=0; i < targets_.size(); ++i)
        {
            const double u = norm(targets_[i] - positions_[i]);
            if (u > error)
            {
                error = u;
            }
        }

        ++iter;
        rel_error = ( std::abs(error - error_prev) / error_prev );

        printf("Pose fitting iteration %d | E = %.6f | rel. E = %.6f/%f\n", iter, error, rel_error,conv_val);
    }
    while(rel_error > conv_val  && iter < max_iter);


    const Vec3f p_after = skeleton.root_->get_global_translation();
    const float y_offset_after = p_after[1] - p_before[1];

    std::cout << y_offset_after << std::endl;
    transform_points(Mat4f::translate(Vec3f(0.0f,y_offset_after,0.0f)));
}
*/
//-----------------------------------------------------------------------------

void IKPose_fit::build_ik_skeleton_recursive(character::Joint *joint)
{

    size_t i;

    ik::Mat4f eigen_mat;
    Mat4f mat;
    ik::Joint* parent_joint;
    ik::Joint* new_joint;

    if (joint->parent_ != NULL)
    {
        parent_joint = ik_skeleton_->getJoint(joint->parent_->get_name());
        new_joint = new ik::Joint(joint->get_name(), parent_joint);
        parent_joint->addChild(new_joint);
        new_joint->setParent(parent_joint);

        ik_skeleton_->addJoint(new_joint);
    }
    else
    {
        new_joint = new ik::Joint(joint->get_name());
        ik_skeleton_->setRoot(new_joint);
    }

    int m,n;
    mat = joint->local_;
    for (m=0; m < 4; ++m)
    {
        for (n=0; n < 4; ++n)
        {
            eigen_mat(n,m) = mat(n,m);
        }

    }

    new_joint->setTransformation(eigen_mat);


    for (i=0; i < joint->children_.size(); ++i)
    {
        build_ik_skeleton_recursive(joint->children_[i]);
    }
}

//-----------------------------------------------------------------------------

void IKPose_fit::create_mapping()
{
    if (!data_->ready())
    {
        return;
    }

    int idx(0);
    const ik::Vec3f X_AXIS(1.0f,0.0f,0.0f);
    const ik::Vec3f Y_AXIS(0.0f,1.0f,0.0f);
    const ik::Vec3f Z_AXIS(0.0f,0.0f,1.0f);

    ik::Mapping mapping;

    size_t i;
    for (i=0; i < data_->free_joints().size(); ++i)
    {/*
        if (data_->free_joints()[i].find("ForeArm") != std::string::npos)
        {
            mapping.map(idx++, data_->free_joints()[i], X_AXIS, ik::ROTATION_AXIS);
            mapping.map(idx++, data_->free_joints()[i], Y_AXIS, ik::ROTATION_AXIS);
            //mapping.map(idx++, data_->free_joints()[i], Z_AXIS, ik::ROTATION_AXIS);
        }
        else*/
        {
            if (data_->joint_limits().count(data_->free_joints()[i]) > 0)
            {
                const std::pair<Vec3f, Vec3f>& limits = data_->joint_limits()[data_->free_joints()[i]];
                const Vec3f& min_angles = limits.first;
                const Vec3f& max_angles = limits.second;
                mapping.map(idx++, data_->free_joints()[i], X_AXIS, ik::ROTATION_AXIS, min_angles[0], max_angles[0]);
                mapping.map(idx++, data_->free_joints()[i], Y_AXIS, ik::ROTATION_AXIS, min_angles[1], max_angles[1]);
                mapping.map(idx++, data_->free_joints()[i], Z_AXIS, ik::ROTATION_AXIS, min_angles[2], max_angles[2]);
                //std::cout << "Joint limits for " << data_->free_joints()[i] << ": min " << min_angles << " ... max " << max_angles << std::endl;
            }
            else
            {
                mapping.map(idx++, data_->free_joints()[i], X_AXIS, ik::ROTATION_AXIS);
                mapping.map(idx++, data_->free_joints()[i], Y_AXIS, ik::ROTATION_AXIS);
                mapping.map(idx++, data_->free_joints()[i], Z_AXIS, ik::ROTATION_AXIS);
            }
        }
    }

    mapping.setPCAEnabled(false);

    ik_skeleton_->setMapping(mapping);
}

//-----------------------------------------------------------------------------

void IKPose_fit::set_targets_and_positions(const std::vector<Vec3f> &targets, const std::vector<Vec3f> &positions)
{
    if (targets.size() != positions.size())
    {
        std::cerr << "IKPose_fit::set_targets_and_positions: [ERROR] #targets does not match #positions." << std::endl;
        return;
    }

    size_t i;
    int j;
    ik::Vec3f target, position, joint_position,joint_to_skin_vec;
    ik::Joint *ik_joint,*closest_ik_joint;
    float dist,closest_dist;
    Vec3f v;

    ik_skeleton_->clearEffectors();

    for (i=0; i < targets.size(); ++i)
    {
        target[0] = targets[i][0];target[1] = targets[i][1];target[2] = targets[i][2];
        position[0] = positions[i][0];position[1] = positions[i][1];position[2] = positions[i][2];



        //find joint closest to
        closest_ik_joint = NULL;
        closest_dist = FLT_MAX;
        dist = 0.0f;
        const Vec3f& pos = positions[i];
        for (j=0; j < (int)ik_skeleton_->getJoints_().size(); ++j)
        {
            ik_joint = ik_skeleton_->getJoints_()[j];

            joint_position = ik_joint->getGlobalTranslation();
            v[0] = joint_position[0];
            v[1] = joint_position[1];
            v[2] = joint_position[2];
            dist = norm(v - pos);

            if (dist < closest_dist)
            {
                closest_ik_joint = ik_joint;
                closest_dist = dist;
            }
        }

        if (closest_ik_joint != NULL)
        {
            //std::cout << closest_ik_joint->getName() << std::endl;
            joint_to_skin_vec = position - closest_ik_joint->getGlobalTranslation();
            //std::cout << joint_to_skin_vec << std::endl;
            ik_skeleton_->addEffector(new ik::Effector(closest_ik_joint, closest_ik_joint->getGlobalTranslation() + joint_to_skin_vec, target));
        }
    }
}

//-----------------------------------------------------------------------------

void IKPose_fit::solve()
{
    for (int i=0; i < 1; ++i)
    {
        ik_solver_->solve(*ik_skeleton_);
    }

    //copy_pose();
}

//-----------------------------------------------------------------------------

void IKPose_fit::copy_pose()
{

    ik::Mat4f eigen_mat;
    Mat4f mat;
    character::Joint* joint;
    ik::Joint *ik_joint;

    int m,n;
    for (size_t i = 0; i < ik_joints_.size(); ++i)
    {
        joint = data_->character()->skeleton().joints_[i];
        ik_joint = ik_joints_[i];
        eigen_mat = ik_joint->getLocalTransformation();

        //copy matrix
        for (m=0; m < 4; ++m) {
            for (n = 0; n < 4; ++n) {
                mat(m,n) = eigen_mat(m,n);
            }
        }

        joint->local_ = mat;

    }
}

//-----------------------------------------------------------------------------

void
IKPose_fit::
transform_points(const Mat4f &M)
{

    if (data_->point_set() == NULL)
    {
        return;
    }

    Mat4f& trans = data_->point_set()->temp_transformation_;
    trans = M * trans;

    // normal matrix
    Mat3f normal_matrix = transpose(inverse(Mat3f(M)));

    // transform full original pointset if available
    for (unsigned int i = 0; i < data_->point_set()->points_.size(); ++ i)
    {
        Point p = data_->point_set()->points_[i];
        p = affine_transform(M, p);
        data_->point_set()->points_[i] = p;

        Point n = data_->point_set()->normals_[i];
        n = normal_matrix * n;
        n.normalize();
        data_->point_set()->normals_[i] = n;
    }
}

//-----------------------------------------------------------------------------

void
IKPose_fit::
transform_points(const Mat4f& M, const std::vector<unsigned int>& indices, std::vector<Point>& result)
{

    result.resize(indices.size());

    // transform points specified by indices
    for (size_t i = 0; i < indices.size(); ++i)
    {
        result[i] = affine_transform(M, data_->point_set()->points_[indices[i]]);
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
