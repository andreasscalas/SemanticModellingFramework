
#include "Pipeline.h"
#include "config/Config.h"


#include <graphene/scene_graph/Object_node.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Landmarks_manager.h>
#include <graphene/surface_mesh/algorithms/posefitting/IKPose_fit.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Template_fit.h>
#include <graphene/pointset_processing/Pointset_processing.h>
#include <graphene/surface_mesh/algorithms/texture_processing/Texture_processing.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/Teeth_correction_helper.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/Eyes_correction_helper.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/Eyelid_fitting_helper.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/Blendshapes_generation.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/Eyes_helper.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/facialfeatures/Facial_features_detection.h>
#include <graphene/character/data_structure/Retargeting.h>
#include <iostream>
#include <fstream>

namespace p2c
{

Pipeline::Pipeline() :
    window_("points2character", 800,800),
    loader_(window_.gl_state(), window_.scene_graph()),
    template_model_(nullptr),
    pointset_body_(nullptr),
    pointset_face_(nullptr),
    pointset_left_hand_(nullptr),
    pointset_right_hand_(nullptr)
{

}

Pipeline::~Pipeline()
{

}

void Pipeline::run()
{
    Stop_watch timer;
    timer.start();

    window_.update();

    //load pointsets and template model
    if (! load())
    {
        window_.set_error_and_run();
    }


    if (pointset_face_ != nullptr)
    {
        //load or select and save landmarks on face pointset
        if (! face_landmarks_selection())
            window_.set_error_and_run();
    }

    if (pointset_left_hand_ != nullptr)
    {
        //load or select and save landmarks on left hand pointset
        if (! left_hand_landmarks_selection())
            window_.set_error_and_run();
    }

    if (pointset_right_hand_ != nullptr)
    {
        //load or select and save landmarks on right hand pointset
        if (! right_hand_landmarks_selection())
            window_.set_error_and_run();
    }


    if (pointset_body_ != nullptr)
    {
        //load or select and save landmarks on body pointset
        if (! body_landmarks_selection())
            window_.set_error_and_run();

        //start fitting of fullbody
        if (! body_fitting())
            window_.set_error_and_run();

        //compute texture for body
        if (! body_texturize())
            window_.set_error_and_run();

        //finalize: inverse posture and set feet on ground
        if (! body_finalize())
            window_.set_error_and_run();
    }


    if (pointset_left_hand_ != nullptr)
    {
        if (! left_hand_fitting())
            window_.set_error_and_run();

        if (! left_hand_texturize())
            window_.set_error_and_run();

        //finalize: inverse posture
        if (! left_hand_finalize())
            window_.set_error_and_run();
    }


    if (pointset_right_hand_ != nullptr)
    {
        if (! right_hand_fitting())
            window_.set_error_and_run();

        if (! right_hand_texturize())
            window_.set_error_and_run();

        //finalize: inverse posture
        if (! right_hand_finalize())
            window_.set_error_and_run();
    }


    if (pointset_face_ != nullptr)
    {
        if (! face_fitting())
            window_.set_error_and_run();

        if (! face_texturize())
            window_.set_error_and_run();
    }


    //correct interior: eyes and teeth
    if (! correct_eyes_and_teeth())
        window_.set_error_and_run();

    //transfer blendshapes
    if (! transfer_blendshapes())
        window_.set_error_and_run();


    Config* cfg = Config::instance();

    //scale to height
    if (cfg->general().dbl_result_height > 0.0)
    {
        template_model_->scale_to_height((float)cfg->general().dbl_result_height, 0.0f);
        window_.update();
    }

    //convert skeleton
    if (cfg->general().b_convert_skeleton)
    {
        character::Retargeting retargeting(template_model_->character());
        if (retargeting.convert_skeleton(cfg->tmpl().fn_skeleton_convert_info))
        {
            template_model_->update_meshes();
            window_.update();
        }
    }

    //retargeting for icspace
    if (cfg->general().b_icspace_retargeting)
    {
        character::Retargeting retargeting(template_model_->character());
        if (!retargeting.apply_retargeting(cfg->tmpl().fn_retargeting_skeleton, cfg->tmpl().fn_retargeting_excluded_joints))
        {
            std::cerr << "Pipeline: [ERROR] Unable to apply ICSPACE retargeting. May have to convert to HANIM first?" << std::endl;
        }
        template_model_->update_meshes();
        window_.update();
    }

    template_model_->update_mesh();
    window_.update();


    if (! save())
        window_.set_error_and_run();

    if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);
    template_model_->set_visible(true);



    const double time = (timer.stop() / 1000.0);
    const double min  = time/60;
    const double sec  = (min - ((int) min)) * 60;

    std::cout << std::endl << "Pipeline: [STATUS] Finished. Time taken: " << (int)min << " min " << (int)sec <<  " s." << std::endl;


    if (Config::instance()->general().b_hidewindow)
    {
        exit(0);
    }

    window_.reset_view();
    window_.set_done_and_run();
}

bool Pipeline::load()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start loading files." << std::endl;


    Config *cfg = Config::instance();

    if (cfg->tmpl().dir_template_db.empty())
    {
        std::cerr << "Pipeline: [ERROR] Template database directory not defined. Unable to proceed." << std::endl;
        return false;
    }

    //load template model
    if (loader_.load(cfg->tmpl().fn_template))
    {
        template_model_ = dynamic_cast<scene_graph::Character_node*>(window_.scene_graph().select_node(cfg->tmpl().fn_template));
        //set main mesh
        template_model_->set_selected_skin(cfg->tmpl().s_bodymesh);

    }
    else// without template we cannot proceed
    {
        std::cerr << "Pipeline: [ERROR] Could not load template model." << std::endl;
        // abort and go into main window loop
        return false;
    }

    //load body point set
    if (loader_.load(cfg->body().fn_pointset))
    {
        pointset_body_ = dynamic_cast<scene_graph::Point_set_node*>(window_.scene_graph().select_node(cfg->body().fn_pointset));
    }

    //load face point set
    if (cfg->face().b_has_face_scan)
    {
        if (loader_.load(cfg->face().fn_pointset))
        {
            pointset_face_ = dynamic_cast<scene_graph::Point_set_node*>(window_.scene_graph().select_node(cfg->face().fn_pointset));
        }
    }

    //load left hand point set
    if (cfg->left_hand().b_has_left_hand_scan)
    {
        if (loader_.load(cfg->left_hand().fn_pointset))
        {
            pointset_left_hand_ = dynamic_cast<scene_graph::Point_set_node*>(window_.scene_graph().select_node(cfg->left_hand().fn_pointset));
        }
    }

    //load right hand point set
    if (cfg->right_hand().b_has_right_hand_scan)
    {
        if (loader_.load(cfg->right_hand().fn_pointset))
        {
            pointset_right_hand_ = dynamic_cast<scene_graph::Point_set_node*>(window_.scene_graph().select_node(cfg->right_hand().fn_pointset));
        }
    }

    //save undeformed state for later use
    save_undeformed_data();


    // if no result filename is specified use default filenames
    if (cfg->general().fn_result.empty())
    {
        if (pointset_face_ != nullptr) //store in face project dir
            cfg->general().fn_result = cfg->face().dir_project + "results/result.bim";
        else //store in body project dir
            cfg->general().fn_result = cfg->body().dir_project + "results/result.bim";
    }

    window_.reset_view();
    window_.update();

    if (pointset_body_ == nullptr && pointset_face_ == nullptr)
    {
        std::cerr << "bool Pipeline: [ERROR] Neither body nor face scan loaded." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}


bool Pipeline::save()
{
    Config* cfg = Config::instance();

    //save result
    if (! template_model_->save(cfg->general().fn_result))
    {
        std::cerr << "Pipeline: [ERROR] Could not save result to filename \"" << Config::instance()->general().fn_result << "\"" << std::endl;
        return false;
    }

    return true;
}

void Pipeline::save_undeformed_data()
{
    Config* cfg = Config::instance();

    Surface_mesh &mesh = template_model_->mesh();

    //save undeformed mesh vertices
    Surface_mesh::Vertex_property<Point> undef_source_prop = mesh.vertex_property<Point>("v:undef_point");
    Surface_mesh::Vertex_property<Point> vpoint            = mesh.vertex_property<Point>("v:point");
    undef_source_prop.vector() = vpoint.vector();

    //save positions of eye joints
    character::Joint* leye_joint = template_model_->character().skeleton().get_joint(cfg->tmpl().s_eyejoint_left.c_str());
    character::Joint* reye_joint = template_model_->character().skeleton().get_joint(cfg->tmpl().s_eyejoint_right.c_str());

    if (leye_joint != nullptr && reye_joint != nullptr)
    {
        Surface_mesh::Mesh_property<Vec3f> undef_leye_pos = mesh.mesh_property<Vec3f>("m:undef_leye_pos");
        Surface_mesh::Mesh_property<Vec3f> undef_reye_pos = mesh.mesh_property<Vec3f>("m:undef_reye_pos");

        undef_leye_pos[0] = leye_joint->get_global_translation();
        undef_reye_pos[0] = reye_joint->get_global_translation();
    }

    //backup texture for luminance adjustment
    if (cfg->general().b_adjust_eye_and_teeth_luminance)
    {
        gl::Texture* colortexture = template_model_->get_texture(gl::TT_COLOR);
        if (colortexture != nullptr)
        {
            Surface_mesh::Mesh_property<gl::Texture> color_texture_backup = mesh.mesh_property<gl::Texture>("m:color_texture_backup");
            colortexture->deep_copy(color_texture_backup[0]);
        }
    }

    //backup original face point set
    if (pointset_face_ != nullptr)
    {
        pointset_face_->backup_point_set();
    }
}

bool Pipeline::body_landmarks_selection()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start body landmark selections." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Landmarks_manager lmm;

    //load landmarks on template
    lmm.load(&template_model_->mesh(), cfg->tmpl().fn_landmarks_body);

    //try to load landmarks on point set
    if (! lmm.load(&pointset_body_->point_set(), cfg->body().fn_landmarks))
    {
        template_model_->set_visible(false);
        if (pointset_body_ != nullptr)       pointset_body_->set_visible(true);
        if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
        if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
        if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

        //manually select landmarks
        if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->body().fn_pointset))
        {
            return false;
        }
        else //save them, if successful
        {
            lmm.save(&pointset_body_->point_set(), cfg->body().fn_landmarks);
        }
    }
    else //we were able to load landmarks
    {
        //still, landmark count could be different -> check landmark count
        if (lmm.n_landmarks_pointset(&pointset_body_->point_set()) != lmm.n_landmarks_template(&template_model_->mesh()))
        {
            pointset_body_->point_set().landmarks_.clear();

            template_model_->set_visible(false);
            if (pointset_body_ != nullptr)       pointset_body_->set_visible(true);
            if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
            if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
            if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

            //manually select landmarks
            if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->body().fn_pointset))
            {
                return false;
            }
            else //save them, if successful
            {
                lmm.save(&pointset_body_->point_set(), cfg->body().fn_landmarks);
            }
        }
    }

    template_model_->set_visible(true);
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(true);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

    window_.reset_view();
    window_.update();
    return true;
}

bool Pipeline::body_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start full body fitting." << std::endl;

    // hide scans exept body scan
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(true);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);
    window_.reset_view();
    window_.update();

    // pose fitting with landmarks
    Config *cfg = Config::instance();

    if (! body_posture_fitting())
    {
        return false;
    }

    const bool with_hands_posture_fitting = false;
    if (with_hands_posture_fitting)
    {
        if (! left_hand_posture_fitting())
        {
            return false;
        }

        if (! right_hand_posture_fitting())
        {
            return false;
        }
    }

    // landmark only fitting
    if (cfg->body().b_landmarks_only_before_finefit)
    {
        body_landmarkonly_fitting();
    }

    //non-rigid fullbody fitting with correspondences
    if (! body_cpc_fitting())
    {
        return false;
    }

    window_.update();

    return true;
}

bool Pipeline::body_posture_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Fitting body posture with landmarks." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Pose_fit_data pose_fit_data;

    if (! pose_fit_data.load_dof(cfg->tmpl().fn_template_ik_dof))
    {
        return false;
    }

    //switch draw mode for "nice" visualization
    template_model_->set_draw_mode("Skeleton");

    //set pose fit data (template and pointset)
    pose_fit_data.set_character(&template_model_->character());
    pose_fit_data.set_point_set(&pointset_body_->point_set());


    //pose fitting with landmarks only
    surface_mesh::IKPose_fit pose_fit(&pose_fit_data);
    pose_fit.set_use_landmarks(true);
    pose_fit.set_use_correspondences(false);
    pose_fit.set_convergence_value(0.05);
    pose_fit.start();

    pointset_body_->update_point_set();
    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    std::cout << std::endl << "Pipeline: [STATUS] Subsampling body point set." << std::endl;

    //subsample point set according to meanedge of template mesh
    if (pointset_face_ != nullptr) //more subsampling when we have an additonal face scan
    {
        Pointset_processing pp(*pointset_body_);
        pp.subsampling_according_to_mesh_meanedge(template_model_->mesh(), 0.75);
    }
    else //preserve more points when we do body fitting only
    {
        Pointset_processing pp(*pointset_body_);
        pp.subsampling_according_to_mesh_meanedge(template_model_->mesh(), 0.5);
    }

    if (cfg->body().b_replace_floor_plane)
    {
        //replace floor plane
        Pointset_processing pp(*pointset_body_);
        pp.replace_biggest_plane_ransac(-0.1f, 0.1f);
    }

    std::cout << std::endl << "Pipeline: [STATUS] Fitting body posture and pca with landmarks and correspondences." << std::endl;

    //pose and pca fitting with correspondences and landmarks
    pose_fit.initial_aligment_with_pca(cfg->tmpl().fn_pca_body_mean_mesh, cfg->tmpl().fn_pca_body, 0.1, 50.0, 2);

    pointset_body_->update_point_set();
    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    return true;
}

void Pipeline::body_landmarkonly_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid body fitting to landmarks only." << std::endl;

    surface_mesh::Template_fit template_fit;
    surface_mesh::Correspondences_settings corr_settings;

    surface_mesh::Energy& E = template_fit.energy();

    surface_mesh::Energy_term* E_reg = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE);
    surface_mesh::Energy_term* E_lm  = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);

    E_reg->set_weight(1.0);
    E_reg->set_multiply_factor(0.5);

    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(1.0);

    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_ANISO_LAPLACE);


    template_fit.set_point_set(&pointset_body_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());
    template_fit.start(corr_settings, 200, 0.05, 0.0001, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();
}



bool Pipeline::body_cpc_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid body fitting to CPC and landmarks (coarse)." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Template_fit template_fit;
    surface_mesh::Correspondences_settings corr_settings;
    surface_mesh::Energy& E = template_fit.energy();

    template_fit.set_point_set(&pointset_body_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    //load vertex weightings

    template_fit.load_vertexweighting(cfg->tmpl().fn_vw_fullbody_hands_fitting, "v:fitting_weight", false);
    template_fit.load_vertexweighting(cfg->tmpl().fn_vw_fullbody_hands_fitting, "v:fitting_weight", true);


    if (pointset_face_ != nullptr) //if we have subsequent face fitting, weight down face region
    {
        template_fit.load_vertexweighting(cfg->tmpl().fn_vw_fullbody_face_fitting, "v:fitting_weight", true);
        //template_fit.load_vertexweighting(cfg->tmpl().fn_vw_fullbody_face_fitting, "v:fitting_weight", true);
    }
    else // otherwise make sure eye deformation is kept to a minimum
    {
        template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_vertices_eyes);
        E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(0.5);
    }
    //shape fitting with point to point correspondences

    //activate energy terms
    surface_mesh::Energy_term* E_reg  = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE);
    E_reg->set_weight(1.0);
    E_reg->set_multiply_factor(0.1);

    surface_mesh::Energy_term* E_lm   = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);
    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(0.5);

    surface_mesh::Energy_term* E_cpc  = E.energy_term_list().get_term(surface_mesh::ET_POINT_TO_POINT);
    E_cpc->set_weight(1.0);
    E_cpc->set_multiply_factor(1.0);

    if (cfg->body().dbl_mouth_shut_weight > 0.0 && pointset_face_ == nullptr)
    {
        //keep mouth shut
        template_fit.load_mouth_shut_selection(cfg->tmpl().fn_sel_mouth_shut);
        surface_mesh::Energy_term* E_ms  = E.energy_term_list().get_term(surface_mesh::ET_MOUTH_SHUT);
        E_ms->set_weight(cfg->body().dbl_mouth_shut_weight);
    }

    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_ANISO_LAPLACE);

    corr_settings.filter_dist     = true;
    corr_settings.dist_until      = 0.05;
    corr_settings.filter_normals  = true;
    corr_settings.normal_until    = 35.0;
    corr_settings.constr_dir      = surface_mesh::corresp_dir_ps2mesh;

    template_fit.start(corr_settings, 200, 0.05, 0.0000001, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    if (cfg->body().int_detail_level >= 1)
    {
        //detailed shape fitting
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid body fitting to CPC and landmarks (fine)." << std::endl;

        corr_settings.dist_until   = 0.01;
        corr_settings.normal_until = 30.0;
        template_fit.start(corr_settings, 200, 0.05, 0.000000001, true, false);

        template_model_->update_mesh();
        window_.reset_view();
        window_.update();
    }

    if (cfg->body().int_detail_level >= 2)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid body fitting to CPC and landmarks (extra fine)." << std::endl;

        corr_settings.dist_until   = 0.0015;
        corr_settings.normal_until = 15.0;
        template_fit.start(corr_settings, 200, 0.05, 0.0000000001, true, false);

        template_model_->update_mesh();
        pointset_body_->set_visible(false);
        window_.update();
    }


    //clear vertex weighting to make sure it does not interfere
    //with further fitting steps
    template_fit.clear_vertexweighting("v:fitting_weight");

    return true;
}


bool Pipeline::body_texturize()
{
    if (! Config::instance()->body().b_textured)
        return true;

    std::cout << std::endl << "Pipeline: [STATUS] Texturizing body." << std::endl;

    Config *cfg = Config::instance();
    utility::Texture_processing texture_processing;

    //apply inverse transformation of point set to template mesh
    template_model_->apply_transform(inverse(pointset_body_->point_set().temp_transformation_));
    //save it
    template_model_->save(cfg->body().fn_invtrans_mesh);
    //and transform back again
    template_model_->apply_transform(pointset_body_->point_set().temp_transformation_);
    window_.update();

    if (Config::instance()->body().b_provide_texture_manually)
    {
        window_.reset_view();
        std::cout << std::endl << "Pipeline: [STATUS] Please provide a body texture." << std::endl;
        window_.print_message_and_pause("Please provide a body texture here:\n" + cfg->body().fn_texture);
    }
    else
    {
        std::cout << std::endl << "Pipeline: [STATUS] Computing body texture." << std::endl;

        if (! texture_processing.compute_texture_photoscan(
                    cfg->get_photoscan_exec(),
                    cfg->get_photoscan_script_dir(),
                    cfg->body().fn_photoscan_project,
                    cfg->body().fn_invtrans_mesh,
                    cfg->body().fn_texture,
                    4096
                    ))
        {
            std::cerr << "Pipeline: [ERROR] Unable to compute texture via photoscan." << std::endl;
            return false;
        }
    }


    std::cout << std::endl << "Pipeline: [STATUS] Merging body texture." << std::endl;

    if (! texture_processing.texturemerge(
                gl::TT_COLOR,
                cfg->body().fn_texture,
                cfg->tmpl().fn_mask_eyes_and_teeth_inverse,
                std::string(),
                false,
                *template_model_)
            )
    {
        std::cerr << "Pipeline: [ERROR] Problem while merging the generated body texture into the template's texture." << std::endl;
        return false;
    }

    template_model_->set_draw_mode("Textured (Shaded)");

    window_.update();


    if (cfg->body().b_repair_hand_texture)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Repairing hand texture." << std::endl;

        if ( !texture_processing.repair_hand_texture(
                    cfg->tmpl().dir_template_db,
                    cfg->tmpl().fn_mask_hands_comparison,
                    cfg->tmpl().fn_mask_hands_repair,
                    cfg->tmpl().fn_mask_hands_dirichlet,
                    *template_model_ ) )
        {
            std::cerr << "Pipeline: [ERROR] Problem while repairing hand texture." << std::endl;
        }
    }
    window_.update();

    if (cfg->body().b_repair_armpit_texture)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Repairing armpit texture." << std::endl;

        if (! texture_processing.repair_armpit_texture(cfg->tmpl().fn_mask_armpit_repair, *template_model_))
        {
            std::cerr << "Pipeline: [ERROR] Problem while repairing armpit texture." << std::endl;
        }
    }

    window_.update();

    if (cfg->general().b_adjust_eye_and_teeth_luminance)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Adjusting Luminance of eyes and teeth." << std::endl;

        if (! texture_processing.adjust_eye_and_teeth_luminance(
                    cfg->tmpl().fn_mask_face_and_ears,
                    cfg->tmpl().fn_mask_eyes_and_teeth_inverse,
                    true, true,
                    *template_model_ ) )
        {
            std::cerr << "Pipeline: [ERROR] Problem while adjusting luminance of eyes and teeth." << std::endl;
        }
    }

    window_.update();

    return true;
}

bool Pipeline::body_finalize()
{
    std::cout << std::endl << "Pipeline: [STATUS] Finalizing full body fit (Feet and skeleton correction)." << std::endl;

    //correct joint positions
    template_model_->character().correct_joint_positions();

    //inverse pose
    template_model_->character().skeleton().reset_to_initial(true);
    template_model_->character().skeleton().update();
    template_model_->character().apply_skinning_on_CPU(true);

    template_model_->update_mesh();
    window_.update();

    Config* cfg = Config::instance();

    //correct feet
    Template_fit template_fit;
    template_fit.set_point_set(&pointset_body_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    if (! template_fit.load_selection_ground(cfg->tmpl().fn_sel_sole_vertices))
    {
        std::cerr << "Pipeline: [ERROR] Could not load sole vertex selection." << std::endl;
        return false;
    }

    //rigidly translate skeleton and mesh
    Surface_mesh &mesh = template_model_->mesh();

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mgroundvertices
            = mesh.get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");

    Surface_mesh::Vertex_property< Point > vpoint = mesh.get_vertex_property< Point >("v:point");

    const std::vector<Surface_mesh::Vertex> &gv = mgroundvertices[0];

    size_t i;
    float avg_dist=0.0f;

    //compute avg distance of sole vertices to ground
    for (i=0; i < gv.size(); ++i)
    {
        avg_dist += -vpoint[gv[i]][1];
    }
    avg_dist *= 1.0f/gv.size();

    character::Skeleton& skeleton = template_model_->character().skeleton();

    //translate skeleton by avg_dist
    skeleton.root_->local_[13] += avg_dist;
    skeleton.init();
    skeleton.set_current_pose_as_bindpose();

    //translate mesh to ground by adding average distance
    for (i=0; i < vpoint.vector().size(); ++i)
    {
        vpoint.vector()[i][1] += avg_dist;
    }


    template_model_->update_mesh();
    window_.update();

    //rigid stuff end... now project soles non-rigidly to ground

    Energy &E = template_fit.energy();

    Energy_term* E_yzero = E.energy_term_list().get_term(ET_PROJECTION_Y_ZERO);
    E_yzero->set_weight(1.0);

    Energy_term* E_keep = E.energy_term_list().get_term(ET_KEEP_VERTICES);
    E_keep->set_weight(10.0);

    Energy_term* E_reg = E.energy_term_list().get_term(ET_NONLIN_ANISO_LAPLACE);
    E_reg->set_weight(0.00000001);

    if (! template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_sole_vertices_inverse))
    {
        std::cerr << "Pipeline: [ERROR] Could not load keep vertices for sole projection." << std::endl;
        return false;
    }

    //one step fitting
    template_fit.start(Correspondences_settings(), 1, DBL_MAX, 0.00000001, true, true);

    //correct joint positions (make sure feet joints are correctly inside of mesh)
    template_model_->character().correct_joint_positions();

    //inverse pose
    template_model_->character().skeleton().reset_to_initial(true);
    template_model_->character().skeleton().update();
    template_model_->character().apply_skinning_on_CPU(true);

    template_model_->character().set_current_pose_as_bindpose();
    template_model_->update_mesh();
    window_.update();

    return true;
}


bool Pipeline::face_landmarks_selection()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start face landmark/contour selections." << std::endl;

    Config* cfg = Config::instance();

    if (cfg->face().dbl_eyelid_weight > 0.0 && cfg->face().b_manual_eye_contour)
    {
        std::ifstream ifs;

        ifs.open(cfg->face().fn_contour_eyes);
        if (! ifs.is_open())
        {
            surface_mesh::Eyes_helper eyes_helper;
            int result = -1;
            std::cout << "Pipeline: [STATUS] Select eye contours. Press ESC to abort pipeline and any key to restart selection!" << std::endl;
            do
            {
                result = eyes_helper.pick_contour_pixelcoords(cfg->face().fn_frontal_image);
            }
            while(result == 1);

            if (result == -1) // ESC was pressed
            {
                std::cout << "Pipeline: [ERROR] Pipeline aborted. Going into main loop..." << std::endl;
                window_.set_error_and_run();
            }
            else if (result == 0) // right number of contour points selected
            {
                eyes_helper.save_contour_pixelcoords(cfg->face().fn_contour_eyes);
            }
        }
        else
        {
            ifs.close();
        }
    }

    if (cfg->face().dbl_ears_lm_weight > 0.0)
    {
        surface_mesh::Landmarks_manager lmm;

        //load landmarks on template
        lmm.load(&template_model_->mesh(), cfg->tmpl().fn_landmarks_face_ears);

        //try to load landmarks on point set
        if (! lmm.load(&pointset_face_->point_set(), cfg->face().fn_ears_landmarks))
        {
            template_model_->set_visible(false);
            if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
            if (pointset_face_ != nullptr)       pointset_face_->set_visible(true);
            if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
            if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

            //manually select landmarks
            if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->face().fn_pointset))
            {
                return false;
            }
            else //save them, if successful
            {
                lmm.save(&pointset_face_->point_set(), cfg->face().fn_ears_landmarks);
            }
        }
        else //we were able to load landmarks
        {
            //still, landmark count could be different -> check landmark count
            if (lmm.n_landmarks_pointset(&pointset_face_->point_set()) != lmm.n_landmarks_template(&template_model_->mesh()))
            {
                template_model_->set_visible(false);
                if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
                if (pointset_face_ != nullptr)       pointset_face_->set_visible(true);
                if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
                if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

                pointset_face_->point_set().landmarks_.clear();

                //manually select landmarks
                if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->face().fn_pointset))
                {
                    return false;
                }
                else //save them, if successful
                {
                    lmm.save(&pointset_face_->point_set(), cfg->face().fn_ears_landmarks);
                }
            }
        }
    }

    //reset state
    template_model_->set_visible(true);
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(true);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);
    window_.reset_view();
    window_.update();

    return true;
}


bool Pipeline::face_fitting()
{
    // hide scans exept face scan
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(true);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);
    window_.reset_view();
    window_.update();

    if (! face_detect_facial_features())
        return false;

    if (! face_initial_alignment())
        return false;

    if (! face_cpc_fitting())
        return false;

    return true;
}

bool Pipeline::face_detect_facial_features()
{
    std::cout << std::endl << "Pipeline: [STATUS] Detecting facial features." << std::endl;

    Config* cfg = Config::instance();
    Landmarks_manager lmm;

    bool have_landmarks = lmm.load(&pointset_face_->point_set(), cfg->face().fn_landmarks_facialfeatures);

    std::ifstream ifs(cfg->face().fn_contour_eyes);
    bool have_eye_contour = ifs.is_open();
    ifs.close();

    // we already have mapped the landmarks and eye contour file is present
    if (have_landmarks && have_eye_contour)
    {
        std::cout << "Pipeline: [STATUS] Loaded landmarks for face pointset from file: \"" << cfg->face().fn_landmarks_facialfeatures << "\"" << std::endl;
        return true;
    }

    surface_mesh::Facial_features_detection ffd;

    // load the detector's model
    if (! ffd.load_model(cfg->tmpl().fn_facialfeatures_model, surface_mesh::Facial_features_detection::FFD_DLIB))
    {
        std::cerr << "Pipeline: [ERROR] Unable to load face detection model from file \"" << cfg->tmpl().fn_facialfeatures_model << "\"" << std::endl;
        return false;
    }

    //detect the facial features
    if (! ffd.detect_ff(cfg->face().fn_frontal_image, surface_mesh::Facial_features_detection::FFD_DLIB))
    {
        std::cerr << "Pipeline: [ERROR] Unable to detect facial features in image \"" << cfg->face().fn_frontal_image << "\"" << std::endl;
        return false;
    }


    //are we using the facial features for eye contour fitting?
    if (! cfg->face().b_manual_eye_contour)
    {
        //save eye contour points of detected facial features
        if (! ffd.save_eye_features(cfg->face().fn_contour_eyes, surface_mesh::Facial_features_detection::FFD_DLIB))
        {
            std::cerr << "Pipeline: [ERROR] Unable to save eye features." << std::endl;
            return false;
        }
    }


    // have we already successfully loaded mapped landmarks?
    if (! have_landmarks)
    {
        //map facial features to pointset
        if (! ffd.map_ff_to_pointset(pointset_face_->point_set(), cfg->face().fn_frontal_image, cfg->face().fn_cameras, surface_mesh::Facial_features_detection::FFD_DLIB))
        {
            std::cerr << "Pipeline: [ERROR] Unable to map facial features to pointset." << std::endl;
            return false;
        }

        lmm.save(&pointset_face_->point_set(), cfg->face().fn_landmarks_facialfeatures);
    }


    pointset_face_->update_point_set();
    window_.update();

    return true;
}

bool Pipeline::face_initial_alignment()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start initial alignment of face." << std::endl;

    Landmarks_manager lmm;

    //load landmarks on template
    lmm.load(&template_model_->mesh(), Config::instance()->tmpl().fn_landmarks_facialfeatures);

    Template_fit template_fit;

    template_fit.set_point_set(&pointset_face_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    //initial aligment with landmarks
    std::cout << "Pipeline: [STATUS] Initial alignment of face (landmarks only)." << std::endl;
    if (! template_fit.do_similarity_transformation_lm(trans_dir_ps2mesh))
    {
        std::cerr << "Pipeline: [ERROR] Unable to perform initial alignment with facial features." << std::endl;
        return false;
    }

    //load ears landmarks here to make sure they are correctly handled while subsampling
    if (Config::instance()->face().dbl_ears_lm_weight > 0.0)
    {
        bool b1 = template_fit.load_ear_landmarks_pointset(Config::instance()->face().fn_ears_landmarks);
        bool b2 = template_fit.load_ear_landmarks_template(Config::instance()->tmpl().fn_landmarks_face_ears);
        if (!b1 || !b2)
        {
            std::cerr << "Pipeline: [ERROR] Unable to load ear landmarks." << std::endl;
            return false;
        }
    }

    std::cout << std::endl << "Pipeline: [STATUS] Subsampling face point set." << std::endl;
    Pointset_processing pp(*pointset_face_);
    pp.subsampling_according_to_mesh_meanedge(template_model_->mesh(), 0.2);

    pointset_face_->update_point_set();


    window_.reset_view();
    window_.update();


    //initial aligment with closest point correspondences
    std::cout << "Pipeline: [STATUS] Initial alignment of face (cpc)." << std::endl;

    Correspondences_settings corr_settings;
    corr_settings.filter_dist = true;
    corr_settings.dist_until  = 0.005;
    corr_settings.filter_normals = true;
    corr_settings.normal_until   = 30.0;
    corr_settings.constr_dir     = corresp_dir_ps2mesh;

    double relative_change = DBL_MAX;
    do
    {
        if (! template_fit.do_similarity_transformation_cpc(trans_dir_ps2mesh, corr_settings, relative_change) )
            return false;
        std::cout << "Pipeline: [STATUS] Relative change of initial alignment with CPC: " << relative_change << std::endl;
        pointset_face_->update_point_set();
        window_.update();
    }
    while (relative_change > 0.005);

    pointset_face_->update_point_set();

    return true;
}


bool Pipeline::face_cpc_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid face fitting." << std::endl;

    Config *cfg = Config::instance();

    Template_fit template_fit;
    template_fit.set_point_set(&pointset_face_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());


    //load vertex weighting
    template_fit.load_vertexweighting(cfg->tmpl().fn_vw_face_and_ears_fitting, "v:fitting_weight", false);

    Energy &E = template_fit.energy();

    Correspondences_settings corr_set;
    corr_set.filter_dist    = true;
    corr_set.dist_until     = 0.01;
    corr_set.filter_normals = true;
    corr_set.normal_until   = 20.0;


    if (cfg->face().dbl_mouth_shut_weight > 0.0)
    {
        Energy_term* E_mouthshut = E.energy_term_list().get_term(ET_MOUTH_SHUT);
        template_fit.load_mouth_shut_selection(cfg->tmpl().fn_sel_mouth_shut);
        E_mouthshut->set_weight(cfg->face().dbl_mouth_shut_weight);
    }

    //perform eye lid fitting if weight is set
    if (cfg->face().dbl_eyelid_weight > 0.0)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Computing eye contour target points." << std::endl;

        //compute eye contour target points
        eyelid_fitting_helper::Eyelid_info eyelid_info;
        eyelid_info.s_eyetrans_left             = cfg->tmpl().s_eyetrans_left;
        eyelid_info.s_eyetrans_right            = cfg->tmpl().s_eyetrans_right;
        eyelid_info.fn_eye_proxy                = cfg->tmpl().fn_eye_proxy;
        eyelid_info.s_bodymesh                  = cfg->tmpl().s_bodymesh;
        eyelid_info.fn_cameras                  = cfg->face().fn_cameras;
        eyelid_info.fn_frontal_image            = cfg->face().fn_frontal_image;
        eyelid_info.fn_contour_eyes             = cfg->face().fn_contour_eyes;
        eyelid_info.fn_sel_eyecontour_left      = cfg->tmpl().fn_sel_eyecontour_left;
        eyelid_info.fn_sel_eyecontour_right     = cfg->tmpl().fn_sel_eyecontour_right;
        eyelid_info.pointset_to_mesh_transformation = pointset_face_->point_set().temp_transformation_;

        if (eyelid_fitting_helper::compute_eye_contour_target_points(eyelid_info, pointset_face_->get_point_set_backup(), template_model_->character()))
        {
            //load eye region vertices to omit during fitting
            template_fit.load_selection_to_omit(cfg->tmpl().fn_sel_eye_vertices_to_omit);

            //activate energy
            Energy_term* E_eyelid = E.energy_term_list().get_term(ET_EYES_CONTOUR);
            E_eyelid->set_weight(cfg->face().dbl_eyelid_weight);
        }
        else
        {
            std::cerr << "Pipeline: [WARNING] Unable to compute eye contour target points. Not applying eyelid fitting." << std::endl;
        }

    }
    /*
    else // otherwise make sure eye deformation is kept to a minimum
    {
        template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_vertices_eyes);
        E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(0.5);
    }
    */

    if (cfg->face().dbl_ears_lm_weight > 0.0)
    {
        //activate energy
        Energy_term* E_ears = E.energy_term_list().get_term(ET_EARS_LM);
        E_ears->set_weight(cfg->face().dbl_ears_lm_weight);
    }

    if (cfg->face().dbl_keep_body_vertices_weight > 0.0)
    {
        if (!template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_body_vertices))
        {
            std::cerr << "Pipeline: Unable to load vertices to keep. Filename: \"" << cfg->tmpl().fn_sel_keep_body_vertices << "\"" << std::endl;
            return false;
        }

        Energy_term* E_keep = E.energy_term_list().get_term(ET_KEEP_VERTICES);
        E_keep->set_weight(cfg->face().dbl_keep_body_vertices_weight);
    }

    //setup remaining energies
    Energy_term* E_lm  = E.energy_term_list().get_term(ET_LANDMARKS);
    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(0.1);

    Energy_term* E_cpc = E.energy_term_list().get_term(ET_POINT_TO_POINT);
    E_cpc->set_weight(1.0);
    E_cpc->set_multiply_factor(1.0);

    Energy_term* E_reg = E.energy_term_list().get_term(ET_NONLIN_ANISO_LAPLACE);
    E_reg->set_weight(1.0);
    E_reg->set_multiply_factor(0.1);

    // set lambda termination criteria
    E.energy_term_list().set_term_for_termination_lambda(ET_NONLIN_ANISO_LAPLACE);


    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid face fitting (coarse)." << std::endl;

    template_fit.start(corr_set, 200, 0.05, 0.00001, true, false);

    template_model_->update_mesh();
    window_.update();

    if (cfg->face().int_detail_level >= 1)
    {
        //detailed shape fitting
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid face fitting (fine)." << std::endl;

        template_fit.start(corr_set, 200, 0.05, 0.00000001, true, false);

        template_model_->update_mesh();
        window_.update();
    }

    if (cfg->face().int_detail_level >= 2)
    {
        //disable landmarks
        E_lm->set_weight(0.0);

        //change correspondence settings
        corr_set.dist_until = 0.005;
        corr_set.normal_until = 20.0;

        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid face fitting (extra fine)." << std::endl;

        template_fit.start(corr_set, 200, 0.05, 0.0000000001, true, false);

        template_model_->update_mesh();
        window_.update();
    }

    //clear vertex weighting to make sure it does not interfere
    //with further fitting steps
    template_fit.clear_vertexweighting("v:fitting_weight");

    return true;
}


bool Pipeline::face_texturize()
{
    if (! Config::instance()->face().b_textured)
        return true;

    std::cout << std::endl << "Pipeline: [STATUS] Texturizing face." << std::endl;

    Config* cfg = Config::instance();
    utility::Texture_processing texture_processing;

    //apply inverse transformation of point set to template mesh
    template_model_->apply_transform(inverse(pointset_face_->point_set().temp_transformation_));
    //save it
    template_model_->save(cfg->face().fn_invtrans_mesh);
    //and transform back again
    template_model_->apply_transform(pointset_face_->point_set().temp_transformation_);

    if (Config::instance()->face().b_provide_texture_manually)
    {
        window_.reset_view();
        std::cout << std::endl << "Pipeline: [STATUS] Please provide a face texture." << std::endl;
        window_.print_message_and_pause("Please provide a face texture here:\n" + cfg->face().fn_texture);
    }
    else
    {
        std::cout << std::endl << "Pipeline: [STATUS] Computing face texture." << std::endl;

        if (! texture_processing.compute_texture_photoscan(
                    cfg->get_photoscan_exec(),
                    cfg->get_photoscan_script_dir(),
                    cfg->face().fn_photoscan_project,
                    cfg->face().fn_invtrans_mesh,
                    cfg->face().fn_texture,
                    4096
                    ))
        {
            std::cerr << "Pipeline: [ERROR] Unable to compute texture via photoscan." << std::endl;
            return false;
        }
    }


    std::cout << std::endl << "Pipeline: [STATUS] Merging face texture." << std::endl;

    if (! texture_processing.texturemerge(
                gl::TT_COLOR,
                cfg->face().fn_texture,
                cfg->tmpl().fn_mask_face_and_ears,
                std::string(),
                true,
                *template_model_)
            )
    {
        std::cerr << "Pipeline: [ERROR] Problem while merging the generated face texture into the template's texture." << std::endl;
        return false;
    }

    template_model_->set_draw_mode("Textured (Shaded)");

    window_.update();

    if (cfg->general().b_adjust_iris_color)
    {
        texture_processing.adjust_iris_color(*template_model_, cfg->face().fn_frontal_image, cfg->face().fn_contour_eyes);
    }

    return true;
}


bool Pipeline::left_hand_landmarks_selection()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start left hand landmark selections." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Landmarks_manager lmm;

    //load landmarks on template
    lmm.load(&template_model_->mesh(), cfg->tmpl().fn_landmarks_hand_left);

    //try to load landmarks on point set
    if (! lmm.load(&pointset_left_hand_->point_set(), cfg->left_hand().fn_landmarks))
    {
        template_model_->set_visible(false);
        if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
        if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
        if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(true);
        if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

        //manually select landmarks
        if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->left_hand().fn_pointset))
        {
            return false;
        }
        else //save them, if successful
        {
            lmm.save(&pointset_left_hand_->point_set(), cfg->left_hand().fn_landmarks);
        }
    }
    else //we were able to load landmarks
    {
        //still, landmark count could be different -> check landmark count
        if (lmm.n_landmarks_pointset(&pointset_left_hand_->point_set()) != lmm.n_landmarks_template(&template_model_->mesh()))
        {
            template_model_->set_visible(false);
            if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
            if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
            if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(true);
            if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);

             pointset_left_hand_->point_set().landmarks_.clear();

            //manually select landmarks
            if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->left_hand().fn_pointset))
            {
                return false;
            }
            else //save them, if successful
            {
                lmm.save(&pointset_left_hand_->point_set(), cfg->left_hand().fn_landmarks);
            }
        }
    }

    template_model_->set_visible(true);
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(true);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);
    window_.reset_view();
    window_.update();
    return true;  
}


bool Pipeline::left_hand_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start left hand fitting." << std::endl;

    // hide other scans
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(true);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(false);
    window_.reset_view();
    window_.update();

    //load landmarks on left hand pointset
    if (! left_hand_landmarks_selection())
    {
        return false;
    }

    // pose fitting with landmarks
    Config *cfg = Config::instance();

    if (! left_hand_posture_fitting())
    {
        return false;
    }

    // landmark only fitting
    if (cfg->left_hand().b_landmarks_only_before_finefit)
    {
        left_hand_landmarkonly_fitting();
    }

    //non-rigid left hand fitting with correspondences
    if (! left_hand_cpc_fitting())
    {
        return false;
    }

    window_.update();

    return true;
}


bool Pipeline::left_hand_posture_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Fitting left hand posture with landmarks." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Pose_fit_data pose_fit_data;

    std::string fn_template_ik_dof = cfg->tmpl().fn_template_ik_dof_hand_left;
    if (! pose_fit_data.load_dof(fn_template_ik_dof))
    {
        return false;
    }

    //switch draw mode for "nice" visualization
    template_model_->set_draw_mode("Skeleton");

    //set pose fit data (template and pointset)
    pose_fit_data.set_character(&template_model_->character());
    if (pointset_left_hand_ != nullptr)
    {
        pose_fit_data.set_point_set(&pointset_left_hand_->point_set());
    }
    else
    {
        pose_fit_data.set_point_set(&pointset_body_->point_set());
    }


    //pose fitting with landmarks only
    surface_mesh::IKPose_fit pose_fit(&pose_fit_data);
    pose_fit.set_use_landmarks(true);
    pose_fit.set_use_correspondences(false);
    pose_fit.set_convergence_value(0.05);
    pose_fit.set_lambda(0.1);
    while (!pose_fit.is_converged())
    {
        if (!pose_fit.minimization_step())
        {
            return false;
        }

        if (pointset_left_hand_ != nullptr)
        {
            pointset_left_hand_->update_point_set();
        }
        else
        {
            pointset_body_->update_point_set();
        }
        template_model_->update_mesh();
        window_.reset_view();
        window_.update();
    }

    if (pointset_left_hand_ != nullptr)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Subsampling hand point set." << std::endl;

        //subsample point set according to meanedge of template mesh
        Pointset_processing pp(*pointset_left_hand_);
        pp.subsampling_according_to_mesh_meanedge(template_model_->mesh(), 0.5);

        pointset_left_hand_->update_point_set();
        window_.reset_view();
        window_.update();
    }

    std::cout << std::endl << "Pipeline: [STATUS] Fitting left hand posture with landmarks and closest point correspondences." << std::endl;

    //pose fitting with landmarks and closest point correspondences
    pose_fit.clean_up(); // clean up because we reuse the same pose_fit object
    pose_fit.set_use_landmarks(true);
    pose_fit.set_use_correspondences(true);
    pose_fit.set_max_dist(0.005);
    pose_fit.set_max_normalangle(35.0);
    pose_fit.set_convergence_value(0.05);
    while (!pose_fit.is_converged())
    {
        if (!pose_fit.minimization_step())
        {
            return false;
        }

        if (pointset_left_hand_ != nullptr)
        {
            pointset_left_hand_->update_point_set();
        }
        else
        {
            pointset_body_->update_point_set();
        }
        template_model_->update_mesh();
        window_.reset_view();
        window_.update();
    }

//    std::cout << std::endl << "Pipeline: [STATUS] Fitting left hand posture and pca with landmarks and correspondences." << std::endl;

    //pose and pca fitting with correspondences and landmarks
//    pose_fit.initial_aligment_with_pca(cfg->tmpl().fn_pca_hand_mean_mesh, cfg->tmpl().fn_pca_hand, 0.1, 50.0, 2);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    return true;
}


void Pipeline::left_hand_landmarkonly_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid left hand fitting to landmarks only." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Template_fit template_fit;
    surface_mesh::Correspondences_settings corr_settings;

    surface_mesh::Energy& E = template_fit.energy();

    template_fit.set_point_set(&pointset_left_hand_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    // keep vertices
    template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_nohands_vertices);
    E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(10.0);

    surface_mesh::Energy_term* E_reg = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE);
    surface_mesh::Energy_term* E_lm  = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);

    E_reg->set_weight(1.0);
    E_reg->set_multiply_factor(0.5);

    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(1.0);

    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_ANISO_LAPLACE);


    template_fit.start(corr_settings, 200, 0.05, 0.000001, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();
}


bool Pipeline::left_hand_cpc_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid left hand fitting to CPC and landmarks (coarse)." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Template_fit template_fit;
    surface_mesh::Correspondences_settings corr_settings;
    surface_mesh::Energy& E = template_fit.energy();

    template_fit.set_point_set(&pointset_left_hand_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    //load vertex weightings
    template_fit.load_vertexweighting(cfg->tmpl().fn_vw_forearms_fitting, "v:fitting_weight", false);

    // keep vertices
    template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_nohands_vertices);
    E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(10.0);

    //shape fitting with point to point correspondences

    //activate energy terms
    surface_mesh::Energy_term* E_reg  = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE);
    E_reg->set_weight(0.000001);
    E_reg->set_multiply_factor(0.1);

    surface_mesh::Energy_term* E_lm   = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);
    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(0.5);

    surface_mesh::Energy_term* E_cpc  = E.energy_term_list().get_term(surface_mesh::ET_POINT_TO_POINT);
    E_cpc->set_weight(1.0);
    E_cpc->set_multiply_factor(1.0);

    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_ANISO_LAPLACE);

    corr_settings.filter_dist     = true;
    corr_settings.dist_until      = 0.02;
    corr_settings.filter_normals  = true;
    corr_settings.normal_until    = 35.0;
    corr_settings.constr_dir      = surface_mesh::corresp_dir_ps2mesh;

    template_fit.start(corr_settings, 200, 0.05, 1e-7, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    if (cfg->left_hand().int_detail_level >= 1)
    {
        //detailed shape fitting
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid left hand fitting to CPC and landmarks (fine)." << std::endl;

        corr_settings.dist_until   = 0.01; // note: 0.01 for Metashape / 0.02 for RC
        corr_settings.normal_until = 30.0;
        template_fit.start(corr_settings, 200, 0.05, 1e-8, true, false); // note: 1e-8 for Metashape / 1e-9 for RC

        template_model_->update_mesh();
        pointset_left_hand_->set_visible(false);
        window_.reset_view();
        window_.update();
    }

    if (cfg->left_hand().int_detail_level >= 2)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid left hand fitting to CPC and landmarks (extra fine)." << std::endl;

        corr_settings.dist_until   = 0.0015; // note: 0.0015 for Metashape / 0.01 for RC
        corr_settings.normal_until = 15.0;
        template_fit.start(corr_settings, 200, 0.05, 1e-9, true, false); // note: 1e-9 for Metashape / 1e-10 for RC

        template_model_->update_mesh();
        pointset_left_hand_->set_visible(false);
        window_.reset_view();
        window_.update();
    }


    //clear vertex weighting to make sure it does not interfere
    //with further fitting steps
    template_fit.clear_vertexweighting("v:fitting_weight");

    return true;
}


bool Pipeline::left_hand_texturize()
{
    if (! Config::instance()->left_hand().b_textured)
        return true;

    std::cout << std::endl << "Pipeline: [STATUS] Texturizing left hand." << std::endl;

    Config* cfg = Config::instance();
    utility::Texture_processing texture_processing;

    //apply inverse transformation of point set to template mesh
    template_model_->apply_transform(inverse(pointset_left_hand_->point_set().temp_transformation_));
    //save it
    template_model_->save(cfg->left_hand().fn_invtrans_mesh);
    //and transform back again
    template_model_->apply_transform(pointset_left_hand_->point_set().temp_transformation_);

    if (Config::instance()->left_hand().b_provide_texture_manually)
    {
        window_.reset_view();
        std::cout << std::endl << "Pipeline: [STATUS] Please provide a left hand texture." << std::endl;
        window_.print_message_and_pause("Please provide a left hand texture here:\n" + cfg->left_hand().fn_texture);
    }
    else
    {
        std::cout << std::endl << "Pipeline: [STATUS] Computing left hand texture." << std::endl;
    
        if (! texture_processing.compute_texture_photoscan(
                    cfg->get_photoscan_exec(),
                    cfg->get_photoscan_script_dir(),
                    cfg->left_hand().fn_photoscan_project,
                    cfg->left_hand().fn_invtrans_mesh,
                    cfg->left_hand().fn_texture,
                    6144
                    ))
        {
            std::cerr << "Pipeline: [ERROR] Unable to compute texture via photoscan." << std::endl;
            return false;
        }
    }


    std::cout << std::endl << "Pipeline: [STATUS] Merging left hand texture." << std::endl;

    if ( !texture_processing.merge_hand_texture(
                cfg->left_hand().fn_texture,
                cfg->tmpl().fn_mask_hand_left_repair,
                cfg->tmpl().fn_mask_hands_dirichlet,
                *template_model_ ) )
    {
        std::cerr << "Pipeline: [ERROR] Problem while merging the generated left hand texture into the template's texture." << std::endl;
        return false;
    }

    template_model_->set_draw_mode("Textured (Shaded)");

    window_.update();

    if (cfg->left_hand().b_repair_fingertips_texture)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Repairing left fingertips texture." << std::endl;

        if ( !texture_processing.repair_hand_texture(
                    cfg->tmpl().dir_template_db,
                    cfg->tmpl().fn_mask_hands_comparison,
                    cfg->tmpl().fn_mask_left_fingertips_repair,
                    "",
                    *template_model_ ) )
        {
            std::cerr << "Pipeline: [ERROR] Problem while repairing left fingertips texture." << std::endl;
        }
    }
    window_.update();

    return true;
}


bool Pipeline::left_hand_finalize()
{
    std::cout << std::endl << "Pipeline: [STATUS] Finalizing left hand fit (Skeleton correction)." << std::endl;

    //correct joint positions
    template_model_->character().correct_joint_positions();

    //inverse pose
    template_model_->character().skeleton().reset_to_initial(true);
    template_model_->character().skeleton().update();
    template_model_->character().apply_skinning_on_CPU(true);

    template_model_->character().set_current_pose_as_bindpose();
    template_model_->update_mesh();
    window_.update();

    return true;
}


bool Pipeline::right_hand_landmarks_selection()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start right hand landmark selections." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Landmarks_manager lmm;

    //load landmarks on template
    lmm.load(&template_model_->mesh(), cfg->tmpl().fn_landmarks_hand_right);

    //try to load landmarks on point set
    if (! lmm.load(&pointset_right_hand_->point_set(), cfg->right_hand().fn_landmarks))
    {
        template_model_->set_visible(false);
        if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
        if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
        if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
        if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(true);

        //manually select landmarks
        if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->right_hand().fn_pointset))
        {
            return false;
        }
        else //save them, if successful
        {
            lmm.save(&pointset_right_hand_->point_set(), cfg->right_hand().fn_landmarks);
        }
    }
    else //we were able to load landmarks
    {
        //still, landmark count could be different -> check landmark count
        if (lmm.n_landmarks_pointset(&pointset_right_hand_->point_set()) != lmm.n_landmarks_template(&template_model_->mesh()))
        {
            pointset_right_hand_->point_set().landmarks_.clear();

            template_model_->set_visible(false);
            if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
            if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
            if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
            if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(true);

            //manually select landmarks
            if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->right_hand().fn_pointset))
            {
                return false;
            }
            else //save them, if successful
            {
                lmm.save(&pointset_right_hand_->point_set(), cfg->right_hand().fn_landmarks);
            }
        }
    }

    template_model_->set_visible(true);
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(true);
    window_.reset_view();
    window_.update();
    return true;  
}


bool Pipeline::right_hand_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start right hand fitting." << std::endl;

    // hide other scans
    if (pointset_face_ != nullptr)       pointset_face_->set_visible(false);
    if (pointset_body_ != nullptr)       pointset_body_->set_visible(false);
    if (pointset_left_hand_ != nullptr)  pointset_left_hand_->set_visible(false);
    if (pointset_right_hand_ != nullptr) pointset_right_hand_->set_visible(true);
    window_.reset_view();
    window_.update();

    //load landmarks on right hand pointset
    if (! right_hand_landmarks_selection())
    {
        return false;
    }

    // pose fitting with landmarks
    Config *cfg = Config::instance();

    if (! right_hand_posture_fitting())
    {
        return false;
    }

    // landmark only fitting
    if (cfg->right_hand().b_landmarks_only_before_finefit)
    {
        right_hand_landmarkonly_fitting();
    }

    //non-rigid right hand fitting with correspondences
    if (! right_hand_cpc_fitting())
    {
        return false;
    }

    window_.update();

    return true;
}


bool Pipeline::right_hand_posture_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Fitting right hand posture with landmarks." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Pose_fit_data pose_fit_data;

    std::string fn_template_ik_dof = cfg->tmpl().fn_template_ik_dof_hand_right;
    if (! pose_fit_data.load_dof(fn_template_ik_dof))
    {
        return false;
    }

    //switch draw mode for "nice" visualization
    template_model_->set_draw_mode("Skeleton");

    //set pose fit data (template and pointset)
    pose_fit_data.set_character(&template_model_->character());
    if (pointset_right_hand_ != nullptr)
    {
        pose_fit_data.set_point_set(&pointset_right_hand_->point_set());
    }
    else
    {
        pose_fit_data.set_point_set(&pointset_body_->point_set());
    }


    //pose fitting with landmarks only
    surface_mesh::IKPose_fit pose_fit(&pose_fit_data);
    pose_fit.set_use_landmarks(true);
    pose_fit.set_use_correspondences(false);
    pose_fit.set_convergence_value(0.05);
    pose_fit.set_lambda(0.1);
    while (!pose_fit.is_converged())
    {
        if (!pose_fit.minimization_step())
        {
            return false;
        }

        if (pointset_right_hand_ != nullptr)
        {
            pointset_right_hand_->update_point_set();
        }
        else
        {
            pointset_body_->update_point_set();
        }
        template_model_->update_mesh();
        window_.reset_view();
        window_.update();
    }

    if (pointset_right_hand_ != nullptr)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Subsampling hand point set." << std::endl;

        //subsample point set according to meanedge of template mesh
        Pointset_processing pp(*pointset_right_hand_);
        pp.subsampling_according_to_mesh_meanedge(template_model_->mesh(), 0.5);

        pointset_right_hand_->update_point_set();
        window_.reset_view();
        window_.update();
    }

    std::cout << std::endl << "Pipeline: [STATUS] Fitting right hand posture with landmarks and closest point correspondences." << std::endl;

    //pose fitting with landmarks and closest point correspondences
    pose_fit.clean_up(); // clean up because we reuse the same pose_fit object
    pose_fit.set_use_landmarks(true);
    pose_fit.set_use_correspondences(true);
    pose_fit.set_max_dist(0.005);
    pose_fit.set_max_normalangle(35.0);
    pose_fit.set_convergence_value(0.05);
    while (!pose_fit.is_converged())
    {
        if (!pose_fit.minimization_step())
        {
            return false;
        }

        if (pointset_right_hand_ != nullptr)
        {
            pointset_right_hand_->update_point_set();
        }
        else
        {
            pointset_body_->update_point_set();
        }
        template_model_->update_mesh();
        window_.reset_view();
        window_.update();
    }

//    std::cout << std::endl << "Pipeline: [STATUS] Fitting right hand posture and pca with landmarks and correspondences." << std::endl;

    //pose and pca fitting with correspondences and landmarks
//    pose_fit.initial_aligment_with_pca(cfg->tmpl().fn_pca_hand_mean_mesh, cfg->tmpl().fn_pca_hand, 0.1, 50.0, 2);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    return true;
}


void Pipeline::right_hand_landmarkonly_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid right hand fitting to landmarks only." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Template_fit template_fit;
    surface_mesh::Correspondences_settings corr_settings;

    surface_mesh::Energy& E = template_fit.energy();

    template_fit.set_point_set(&pointset_right_hand_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    // keep vertices
    template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_nohands_vertices);
    E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(10.0);

    surface_mesh::Energy_term* E_reg = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE);
    surface_mesh::Energy_term* E_lm  = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);

    E_reg->set_weight(1.0);
    E_reg->set_multiply_factor(0.5);

    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(1.0);

    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_ANISO_LAPLACE);

    template_fit.start(corr_settings, 200, 0.05, 0.000001, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();
}


bool Pipeline::right_hand_cpc_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid right hand fitting to CPC and landmarks (coarse)." << std::endl;

    Config *cfg = Config::instance();
    surface_mesh::Template_fit template_fit;
    surface_mesh::Correspondences_settings corr_settings;
    surface_mesh::Energy& E = template_fit.energy();

    template_fit.set_point_set(&pointset_right_hand_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    //load vertex weightings
    template_fit.load_vertexweighting(cfg->tmpl().fn_vw_forearms_fitting, "v:fitting_weight", false);

    // keep vertices
    template_fit.load_selection_to_keep(cfg->tmpl().fn_sel_keep_nohands_vertices);
    E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(10.0);

    //shape fitting with point to point correspondences

    //activate energy terms
    surface_mesh::Energy_term* E_reg  = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_ANISO_LAPLACE);
    E_reg->set_weight(0.000001);
    E_reg->set_multiply_factor(0.1);

    surface_mesh::Energy_term* E_lm   = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);
    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(0.5);

    surface_mesh::Energy_term* E_cpc  = E.energy_term_list().get_term(surface_mesh::ET_POINT_TO_POINT);
    E_cpc->set_weight(1.0);
    E_cpc->set_multiply_factor(1.0);

    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_ANISO_LAPLACE);

    corr_settings.filter_dist     = true;
    corr_settings.dist_until      = 0.02;
    corr_settings.filter_normals  = true;
    corr_settings.normal_until    = 35.0;
    corr_settings.constr_dir      = surface_mesh::corresp_dir_ps2mesh;

    template_fit.start(corr_settings, 200, 0.05, 1e-7, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    if (cfg->right_hand().int_detail_level >= 1)
    {
        //detailed shape fitting
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid right hand fitting to CPC and landmarks (fine)." << std::endl;

        corr_settings.dist_until   = 0.01; // note: 0.01 for Metashape / 0.02 for RC
        corr_settings.normal_until = 30.0;
        template_fit.start(corr_settings, 200, 0.05, 1e-8, true, false); // note: 1e-8 for Metashape / 1e-9 for RC

        template_model_->update_mesh();
        pointset_right_hand_->set_visible(false);
        window_.reset_view();
        window_.update();
    }

    if (cfg->right_hand().int_detail_level >= 2)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid right hand fitting to CPC and landmarks (extra fine)." << std::endl;

        corr_settings.dist_until   = 0.0015; // note: 0.0015 for Metashape / 0.01 for RC
        corr_settings.normal_until = 15.0;
        template_fit.start(corr_settings, 200, 0.05, 1e-9, true, false); // note: 1e-9 for Metashape / 1e-10 for RC

        template_model_->update_mesh();
        pointset_right_hand_->set_visible(false);
        window_.reset_view();
        window_.update();
    }


    //clear vertex weighting to make sure it does not interfere
    //with further fitting steps
    template_fit.clear_vertexweighting("v:fitting_weight");

    return true;
}


bool Pipeline::right_hand_texturize()
{
    if (! Config::instance()->right_hand().b_textured)
        return true;

    std::cout << std::endl << "Pipeline: [STATUS] Texturizing right hand." << std::endl;

    Config* cfg = Config::instance();
    utility::Texture_processing texture_processing;

    //apply inverse transformation of point set to template mesh
    template_model_->apply_transform(inverse(pointset_right_hand_->point_set().temp_transformation_));
    //save it
    template_model_->save(cfg->right_hand().fn_invtrans_mesh);
    //and transform back again
    template_model_->apply_transform(pointset_right_hand_->point_set().temp_transformation_);

    if (Config::instance()->right_hand().b_provide_texture_manually)
    {
        window_.reset_view();
        std::cout << std::endl << "Pipeline: [STATUS] Please provide a right hand texture." << std::endl;
        window_.print_message_and_pause("Please provide a right hand texture here:\n" + cfg->right_hand().fn_texture);
    }
    else
    {
        std::cout << std::endl << "Pipeline: [STATUS] Computing right hand texture." << std::endl;

        if (! texture_processing.compute_texture_photoscan(
                    cfg->get_photoscan_exec(),
                    cfg->get_photoscan_script_dir(),
                    cfg->right_hand().fn_photoscan_project,
                    cfg->right_hand().fn_invtrans_mesh,
                    cfg->right_hand().fn_texture,
                    6144
                    ))
        {
            std::cerr << "Pipeline: [ERROR] Unable to compute texture via photoscan." << std::endl;
            return false;
        }
    }

    std::cout << std::endl << "Pipeline: [STATUS] Merging right hand texture." << std::endl;

    if ( !texture_processing.merge_hand_texture(
                cfg->right_hand().fn_texture,
                cfg->tmpl().fn_mask_hand_right_repair,
                cfg->tmpl().fn_mask_hands_dirichlet,
                *template_model_ ) )
    {
        std::cerr << "Pipeline: [ERROR] Problem while merging the generated right hand texture into the template's texture." << std::endl;
        return false;
    }

    template_model_->set_draw_mode("Textured (Shaded)");

    window_.update();

    if (cfg->right_hand().b_repair_fingertips_texture)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Repairing right fingertips texture." << std::endl;

        if ( !texture_processing.repair_hand_texture(
                    cfg->tmpl().dir_template_db,
                    cfg->tmpl().fn_mask_hands_comparison,
                    cfg->tmpl().fn_mask_right_fingertips_repair,
                    "",
                    *template_model_ ) )
        {
            std::cerr << "Pipeline: [ERROR] Problem while repairing right fingertips texture." << std::endl;
        }
    }
    window_.update();

    return true;
}


bool Pipeline::right_hand_finalize()
{
    std::cout << std::endl << "Pipeline: [STATUS] Finalizing right hand fit (Skeleton correction)." << std::endl;

    //correct joint positions
    template_model_->character().correct_joint_positions();

    //inverse pose
    template_model_->character().skeleton().reset_to_initial(true);
    template_model_->character().skeleton().update();
    template_model_->character().apply_skinning_on_CPU(true);

    template_model_->character().set_current_pose_as_bindpose();
    template_model_->update_mesh();
    window_.update();

    return true;
}


bool Pipeline::correct_eyes_and_teeth()
{
    std::cout << std::endl << "Pipeline: [STATUS] Correcting eyes and teeth." << std::endl;

    Config* cfg = Config::instance();

    //correct teeth
    if (! teeth_correction_helper::correct_teeth(template_model_->character(),
                                cfg->tmpl().s_bodymesh,
                                cfg->tmpl().s_teeth_up,
                                cfg->tmpl().s_teeth_down,
                                cfg->tmpl().fn_sel_mouth_region))
    {
        std::cerr << "Pipeline: [ERROR] Correcting teeth failed." << std::endl;

        return false;
    }


    //correct eyes
    eyes_correction_helper::Eye_info eye_info;

    eye_info.fn_sel_eyeregion_left = cfg->tmpl().fn_sel_eyeregion_left;
    eye_info.fn_sel_eyeregion_right = cfg->tmpl().fn_sel_eyeregion_right;

    eye_info.s_eyemesh_left   = cfg->tmpl().s_eyemesh_left;
    eye_info.s_eyemesh_right  = cfg->tmpl().s_eyemesh_right;
    eye_info.s_eyetrans_left  = cfg->tmpl().s_eyetrans_left;
    eye_info.s_eyetrans_right = cfg->tmpl().s_eyetrans_right;
    eye_info.s_eyegland_left  = cfg->tmpl().s_eyegland_left;
    eye_info.s_eyegland_right = cfg->tmpl().s_eyegland_right;
    eye_info.s_eyejoint_left  = cfg->tmpl().s_eyejoint_left;
    eye_info.s_eyejoint_right = cfg->tmpl().s_eyejoint_right;
    eye_info.s_bodymesh       = cfg->tmpl().s_bodymesh;

    if (! eyes_correction_helper::correct_eyes(template_model_->character(), eye_info))
    {
        std::cerr << "Pipeline: [ERROR] Correcting eyes failed." << std::endl;

        return false;
    }


    //update view
    template_model_->update_meshes();
    window_.update();


    return true;
}

bool Pipeline::transfer_blendshapes()
{
    std::cout << std::endl << "Pipeline: [STATUS] Blendshapes transfer." << std::endl;

    Blendshapes_generation bg;

    if (! bg.transfer_blendshapes(
                template_model_->character(),
                Config::instance()->tmpl().s_bodymesh,
                Config::instance()->tmpl().fn_sel_deftrans_fixed_vertices,
                false))
    {
        std::cout << "Pipeline: [ERROR] Blendshapes transfer failed." << std::endl;
        return false;
    }

    return true;
}


}
