
#include "Pipeline.h"
#include "config/Config.h"


#include <graphene/scene_graph/Object_node.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Landmarks_manager.h>
#include <graphene/surface_mesh/algorithms/templatefitting/Template_fit.h>
#include <graphene/pointset_processing/Pointset_processing.h>
#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/facialfeatures/Facial_features_detection.h>

namespace p2c
{

Pipeline::Pipeline() :
    window_("points2head", 800,800),
    loader_(window_.gl_state(), window_.scene_graph()),
    template_model_(nullptr),
    pointset_head_(nullptr)
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

    if (pointset_head_ != nullptr)
    {
        //load or select and save ear landmarks on head pointset
        if (! head_ear_landmarks_selection())
            window_.set_error_and_run();

        //load or detect or select facial features
        if (!head_landmarks_facial_features())
            window_.set_error_and_run();

        //start fitting of head
        if (! head_fitting())
            window_.set_error_and_run();

        //apply inverse transformation of point set to template mesh
        template_model_->apply_transform(inverse(pointset_head_->point_set().temp_transformation_));
        //save it
        Config *cfg = Config::instance();
        template_model_->save(cfg->head().fn_invtrans_mesh);
        //and transform back again
        template_model_->apply_transform(pointset_head_->point_set().temp_transformation_);
        window_.update();
    }

    if (! save())
        window_.set_error_and_run();

    if (pointset_head_ != nullptr) pointset_head_->set_visible(false);
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
    std::string fn_template = cfg->tmpl().fn_template_head;
    if (loader_.load(fn_template))
    {
        template_model_ = dynamic_cast<scene_graph::Surface_mesh_node*>(window_.scene_graph().select_node(fn_template));
    }
    else// without template we cannot proceed
    {
        std::cerr << "Pipeline: [ERROR] Could not load template model." << std::endl;
        // abort and go into main window loop
        return false;
    }

    //load head point set
    if (loader_.load(cfg->head().fn_pointset))
    {
        pointset_head_ = dynamic_cast<scene_graph::Point_set_node*>(window_.scene_graph().select_node(cfg->head().fn_pointset));
    }

    // if no result filename is specified use default filenames
    if (cfg->general().fn_result.empty())
    {
        //store in head project dir
        cfg->general().fn_result = cfg->head().dir_project + "results/result.obj";
    }

    window_.reset_view();
    window_.update();

    if (pointset_head_ == nullptr)
    {
        std::cerr << "bool Pipeline: [ERROR] Could not load head scan." << std::endl;
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

bool Pipeline::head_ear_landmarks_selection()
{
    Config *cfg = Config::instance();

    if (cfg->head().dbl_ears_lm_weight > 0.0)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Start head ear landmark selections." << std::endl;

        surface_mesh::Landmarks_manager lmm;

        //load ear landmarks on template
        std::string fn_landmarks_face_ears = cfg->tmpl().fn_landmarks_face_ears;
        std::string fn_mapping_body2head = cfg->tmpl().fn_mapping_body2head;
        if (fn_mapping_body2head.empty())
        {
            lmm.load(&template_model_->mesh(), fn_landmarks_face_ears);
        }
        else
        {
            lmm.load(&template_model_->mesh(), fn_landmarks_face_ears, fn_mapping_body2head);
        }

        //try to load landmarks on point set
        if (! lmm.load(&pointset_head_->point_set(), cfg->head().fn_ears_landmarks))
        {
            template_model_->set_visible(false);

            //manually select landmarks
            if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->head().fn_pointset))
            {
                return false;
            }
            else //save them, if successful
            {
                lmm.save(&pointset_head_->point_set(), cfg->head().fn_ears_landmarks);
            }
        }
        else //we were able to load landmarks
        {
            //still, landmark count could be different -> check landmark count
            if (lmm.n_landmarks_pointset(&pointset_head_->point_set()) != lmm.n_landmarks_template(&template_model_->mesh()))
            {
                template_model_->set_visible(false);
                pointset_head_->point_set().landmarks_.clear();

                //manually select landmarks
                if (!window_.select_n_landmarks(lmm.n_landmarks_template(&template_model_->mesh()), cfg->head().fn_pointset))
                {
                    return false;
                }
                else //save them, if successful
                {
                    lmm.save(&pointset_head_->point_set(), cfg->head().fn_ears_landmarks);
                }
            }
        }

        template_model_->set_visible(true);
        window_.reset_view();
        window_.update();
    }

    return true;
}

bool Pipeline::head_landmarks_facial_features()
{
    std::cout << std::endl << "Pipeline: [STATUS] Selecting or detecting facial features." << std::endl;

    Config* cfg = Config::instance();
    Landmarks_manager lmm;

    bool have_landmarks = lmm.load(&pointset_head_->point_set(), cfg->head().fn_landmarks_facialfeatures);

    // we already have landmarks
    if (have_landmarks)
    {
        std::cout << "Pipeline: [STATUS] Loaded landmarks for head pointset from file: \"" << cfg->head().fn_landmarks_facialfeatures << "\"" << std::endl;
        return true;
    }

    // we have a frontal photograph and a camera-file, i.e., detect facial features
    if (!cfg->head().fn_frontal_image.empty() && !cfg->head().fn_cameras.empty())
    {
        surface_mesh::Facial_features_detection ffd;

        // load the detector's model
        if (! ffd.load_model(cfg->tmpl().fn_facialfeatures_model, surface_mesh::Facial_features_detection::FFD_DLIB))
        {
            std::cerr << "Pipeline: [ERROR] Unable to load face detection model from file \"" << cfg->tmpl().fn_facialfeatures_model << "\"" << std::endl;
            return false;
        }

        //detect the facial features
        if (! ffd.detect_ff(cfg->head().fn_frontal_image, surface_mesh::Facial_features_detection::FFD_DLIB))
        {
            std::cerr << "Pipeline: [ERROR] Unable to detect facial features in image \"" << cfg->head().fn_frontal_image << "\"" << std::endl;
            return false;
        }

        // have we already successfully loaded mapped landmarks?
        if (! have_landmarks)
        {
            //map facial features to pointset
            if (! ffd.map_ff_to_pointset(pointset_head_->point_set(), cfg->head().fn_frontal_image, cfg->head().fn_cameras, surface_mesh::Facial_features_detection::FFD_DLIB))
            {
                std::cerr << "Pipeline: [ERROR] Unable to map facial features to pointset." << std::endl;
                return false;
            }

            lmm.save(&pointset_head_->point_set(), cfg->head().fn_landmarks_facialfeatures);
        }
    }
    // manually select facial features
    else
    {
        surface_mesh::Landmarks_manager l_lmm;

        //load facial features landmarks on template
        std::string fn_landmarks_facialfeatures = cfg->tmpl().fn_landmarks_facialfeatures;
        std::string fn_mapping_body2head = cfg->tmpl().fn_mapping_body2head;
        if (fn_mapping_body2head.empty())
        {
            l_lmm.load(&template_model_->mesh(), fn_landmarks_facialfeatures);
        }
        else
        {
            l_lmm.load(&template_model_->mesh(), fn_landmarks_facialfeatures, fn_mapping_body2head);
        }

        //try to load landmarks on point set
        if (! l_lmm.load(&pointset_head_->point_set(), cfg->head().fn_landmarks_facialfeatures))
        {
            template_model_->set_visible(false);

            //manually select landmarks
            if (!window_.select_n_landmarks(l_lmm.n_landmarks_template(&template_model_->mesh()), cfg->head().fn_pointset))
            {
                return false;
            }
            else //save them, if successful
            {
                l_lmm.save(&pointset_head_->point_set(), cfg->head().fn_landmarks_facialfeatures);
            }
        }
        else //we were able to load landmarks
        {
            //still, landmark count could be different -> check landmark count
            if (l_lmm.n_landmarks_pointset(&pointset_head_->point_set()) != l_lmm.n_landmarks_template(&template_model_->mesh()))
            {
                template_model_->set_visible(false);
                pointset_head_->point_set().landmarks_.clear();

                //manually select landmarks
                if (!window_.select_n_landmarks(l_lmm.n_landmarks_template(&template_model_->mesh()), cfg->head().fn_pointset))
                {
                    return false;
                }
                else //save them, if successful
                {
                    l_lmm.save(&pointset_head_->point_set(), cfg->head().fn_landmarks_facialfeatures);
                }
            }
        }
    }

    pointset_head_->update_point_set();
    window_.update();

    return true;
}

bool Pipeline::head_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start full head fitting." << std::endl;

    Config *cfg = Config::instance();

    // initial alignment
// TEMPORARILY TURNED OFF    if (! head_initial_alignment())
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        return false;
// TEMPORARILY TURNED OFF    }

    //non-rigid head fitting with correspondences
    if (! head_cpc_fitting())
    {
        return false;
    }

    window_.update();

    return true;
}

bool Pipeline::head_initial_alignment()
{
    std::cout << std::endl << "Pipeline: [STATUS] Start initial alignment of head." << std::endl;

    Config *cfg = Config::instance();
    Landmarks_manager lmm;

    //load landmarks on template
    std::string fn_landmarks_facialfeatures = cfg->tmpl().fn_landmarks_facialfeatures;
    std::string fn_mapping_body2head = cfg->tmpl().fn_mapping_body2head;
    if (fn_mapping_body2head.empty())
    {
        lmm.load(&template_model_->mesh(), fn_landmarks_facialfeatures);
    }
    else
    {
        lmm.load(&template_model_->mesh(), fn_landmarks_facialfeatures, fn_mapping_body2head);
    }

    Template_fit template_fit;

    template_fit.set_point_set(&pointset_head_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    // load head pca model
    if (! template_fit.get_pca().load_pca_model(cfg->tmpl().fn_pca_head))
    {
        std::cerr << "Pipeline: [ERROR] Unable to load head pca model." << std::endl;
        return false;
    }

    //initial aligment with landmarks
    std::cout << "Pipeline: [STATUS] Initial alignment of head (landmarks only)." << std::endl;
    if (! template_fit.do_similarity_transformation_lm(trans_dir_ps2mesh))
    {
        std::cerr << "Pipeline: [ERROR] Unable to perform initial alignment with facial features." << std::endl;
        return false;
    }

    //load ears landmarks here to make sure they are correctly handled while subsampling
    if (cfg->head().dbl_ears_lm_weight > 0.0)
    {
        //load ear landmarks on pointset
        bool b1 = template_fit.load_ear_landmarks_pointset(cfg->head().fn_ears_landmarks);

        //load ear landmarks on template
        std::string fn_landmarks_face_ears = cfg->tmpl().fn_landmarks_face_ears;
        std::string fn_mapping_body2head = cfg->tmpl().fn_mapping_body2head;
        bool b2 = false;
        if (fn_mapping_body2head.empty())
        {
            b2 = template_fit.load_ear_landmarks_template(fn_landmarks_face_ears);
        }
        else
        {
            b2 = template_fit.load_ear_landmarks_template(fn_landmarks_face_ears, fn_mapping_body2head);
        }

        if (!b1 || !b2)
        {
            std::cerr << "Pipeline: [ERROR] Unable to load ear landmarks." << std::endl;
            return false;
        }
    }

    std::cout << std::endl << "Pipeline: [STATUS] Subsampling head point set." << std::endl;
    Pointset_processing pp(*pointset_head_);
    pp.subsampling_according_to_mesh_meanedge(template_model_->mesh(), 0.25);

    pointset_head_->update_point_set();

    window_.reset_view();
    window_.update();

    // settings for closest point correspondences
    Correspondences_settings corr_settings;
    corr_settings.filter_dist = true;
    corr_settings.dist_until  = 0.005; // 0.02
    corr_settings.filter_normals = true;
    corr_settings.normal_until   = 30.0; // 45.0
    corr_settings.constr_dir     = corresp_dir_ps2mesh;

    /*
    //initial aligment with closest point correspondences
    std::cout << "Pipeline: [STATUS] Initial alignment of head (cpc)." << std::endl;

    double relative_change = DBL_MAX;
    do
    {
        if (! template_fit.do_similarity_transformation_cpc(trans_dir_ps2mesh, corr_settings, relative_change) )
            return false;
        std::cout << "Pipeline: [STATUS] Relative change of initial alignment with CPC: " << relative_change << std::endl;
        pointset_head_->update_point_set();
        window_.update();
    }
    while (relative_change > 0.005);

    pointset_head_->update_point_set();

    window_.reset_view();
    window_.update();
    */

    // load pca mean and set as template
    Surface_mesh mesh_pca_mean;
    mesh_pca_mean.read(cfg->tmpl().fn_pca_head_mean_mesh);
    if (0 == mesh_pca_mean.n_vertices())
    {
        std::cerr << "Pipeline: [ERROR] PCA mesh has no vertices." << std::endl;
        return false;
    }
    template_fit.set_template_mesh(&mesh_pca_mean);

    // get landmarks from (previous) head template
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks_character_template
        = template_model_->mesh().get_mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    if (!landmarks_character_template)
    {
        std::cerr << "Pipeline: [ERROR] Can't get landmarks from head template." << std::endl;
        return false;
    }
    std::vector<Surface_mesh::Vertex>& landmark_character_template_vec = landmarks_character_template[0];

    // set landmarks for mesh_pca_mean
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > landmarks_pca_mesh
        = mesh_pca_mean.mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    std::vector<Surface_mesh::Vertex>& landmark_pca_mesh_vec = landmarks_pca_mesh[0];
    landmark_pca_mesh_vec.resize(landmark_character_template_vec.size());
    for (size_t i = 0; i < landmark_character_template_vec.size(); ++i)
    {
        Surface_mesh::Vertex& character_v = landmark_character_template_vec[i];

        landmark_pca_mesh_vec[i] = Surface_mesh::Vertex(character_v.idx());
    }

    //initial aligment with landmarks
    std::cout << "Pipeline: [STATUS] Initial alignment of head (landmarks only)." << std::endl;
    if (! template_fit.do_similarity_transformation_lm(trans_dir_ps2mesh))
    {
        std::cerr << "Pipeline: [ERROR] Unable to perform initial alignment with facial features." << std::endl;
        return false;
    }

    surface_mesh::Energy& E = template_fit.energy();

    //activate/deactivate energy terms and set weights
    surface_mesh::Energy_term* E_pca_lm = E.energy_term_list().get_term(surface_mesh::ET_PCA_LANDMARKS);
    E_pca_lm->set_weight(1.0);
    surface_mesh::Energy_term* E_pca_cpc_ps2mesh = E.energy_term_list().get_term(surface_mesh::ET_PCA_CPC_PS2MESH);
    E_pca_cpc_ps2mesh->set_weight(0.0);
    surface_mesh::Energy_term* E_pca_tik = E.energy_term_list().get_term(surface_mesh::ET_TIKHONOV_PCA);
    E_pca_tik->set_weight(0.00002);

    //fit one step
    template_fit.start(corr_settings, 1, DBL_MAX, 1e-7, true, true);

    //alternatingly:
    //1) register point set to pca-fit and
    //2) do pca-fit based on CPC until convergence
    const bool refine_with_CPC = true;
    if (refine_with_CPC)
    {
        for (unsigned int i = 0; i < 5; ++i)
        {
            double relative_change;
            if (!template_fit.do_similarity_transformation_cpc(trans_dir_ps2mesh, corr_settings, relative_change))
            {
                std::cerr << "Pipeline: [ERROR] Can't do similarity transformation based on CPC." << std::endl;
                return false;
            }
            std::cerr << "Pipeline: [STATUS] relative_change: " << relative_change << std::endl;

            // optimize for pca weights w.r.t. CPC (scan-to-template)
            // enable/disable energy terms
            E_pca_lm->set_weight(0.0);
            E_pca_cpc_ps2mesh->set_weight(1.0);
            E_pca_tik->set_weight(0.00002);

            // fit one step
            template_fit.start(corr_settings, 1, DBL_MAX, 1e-7, true, true);

            if (relative_change <= 0.05)
            {
                break;
            }
        }
    }

    // disable energy terms
    E_pca_lm->set_weight(0.0);
    E_pca_cpc_ps2mesh->set_weight(0.0);
    E_pca_tik->set_weight(0.0);

    // replace vertex positions
    for (auto v : mesh_pca_mean.vertices())
    {
        template_model_->mesh().position(v) = mesh_pca_mean.position(v);
    }

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

//    window_.print_message_and_pause("AFTER PCA");

    return true;
}

bool Pipeline::head_cpc_fitting()
{
    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid head fitting." << std::endl;

    Config *cfg = Config::instance();

    surface_mesh::Template_fit template_fit;
    template_fit.set_point_set(&pointset_head_->point_set());
    template_fit.set_template_mesh(&template_model_->mesh());

    //load vertex weighting
// TEMPORARILY TURNED OFF    std::string fn_vw_face_and_ears_fitting = cfg->tmpl().fn_vw_face_and_ears_fitting;
// TEMPORARILY TURNED OFF    std::string fn_mapping_body2head = cfg->tmpl().fn_mapping_body2head;
// TEMPORARILY TURNED OFF    if (! fn_vw_face_and_ears_fitting.empty())
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        if (fn_mapping_body2head.empty())
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            template_fit.load_vertexweighting(fn_vw_face_and_ears_fitting, "v:fitting_weight", false);
// TEMPORARILY TURNED OFF        }
// TEMPORARILY TURNED OFF        else
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            template_fit.load_vertexweighting(fn_vw_face_and_ears_fitting, fn_mapping_body2head, "v:fitting_weight", false);
// TEMPORARILY TURNED OFF        }
// TEMPORARILY TURNED OFF    }

    surface_mesh::Energy& E = template_fit.energy();

    surface_mesh::Correspondences_settings corr_settings;
    corr_settings.filter_dist     = true;
    corr_settings.dist_until      = 0.02; // 0.02 / 0.01
    corr_settings.filter_normals  = true;
    corr_settings.normal_until    = 35.0; // 45.0 / 20.0
    corr_settings.constr_dir      = surface_mesh::corresp_dir_ps2mesh;

// TEMPORARILY TURNED OFF    if (cfg->head().dbl_mouth_shut_weight > 0.0)
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        std::string fn_sel_mouth_shut = cfg->tmpl().fn_sel_mouth_shut;

// TEMPORARILY TURNED OFF        Energy_term* E_mouth_shut = E.energy_term_list().get_term(surface_mesh::ET_MOUTH_SHUT);
// TEMPORARILY TURNED OFF        if (fn_mapping_body2head.empty())
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            template_fit.load_mouth_shut_selection(fn_sel_mouth_shut);
// TEMPORARILY TURNED OFF        }
// TEMPORARILY TURNED OFF        else
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            template_fit.load_mouth_shut_selection(fn_sel_mouth_shut, fn_mapping_body2head);
// TEMPORARILY TURNED OFF        }

// TEMPORARILY TURNED OFF        E_mouth_shut->set_weight(cfg->head().dbl_mouth_shut_weight);
// TEMPORARILY TURNED OFF    }

// TEMPORARILY TURNED OFF    if (cfg->head().dbl_ears_lm_weight > 0.0)
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        //activate energy
// TEMPORARILY TURNED OFF        Energy_term* E_ears = E.energy_term_list().get_term(surface_mesh::ET_EARS_LM);
// TEMPORARILY TURNED OFF        E_ears->set_weight(cfg->head().dbl_ears_lm_weight);
// TEMPORARILY TURNED OFF    }

// TEMPORARILY TURNED OFF    if (cfg->head().dbl_keep_vertices_weight > 0.0)
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        std::string fn_sel_keep_vertices = cfg->tmpl().fn_sel_keep_vertices;
// TEMPORARILY TURNED OFF        if (fn_mapping_body2head.empty())
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            if (!template_fit.load_selection_to_keep(fn_sel_keep_vertices))
// TEMPORARILY TURNED OFF            {
// TEMPORARILY TURNED OFF                std::cerr << "Pipeline: Unable to load vertices to keep. Filename: \"" << fn_sel_keep_vertices << "\"" << std::endl;
// TEMPORARILY TURNED OFF                return false;
// TEMPORARILY TURNED OFF            }
// TEMPORARILY TURNED OFF        }
// TEMPORARILY TURNED OFF        else
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            if (!template_fit.load_selection_to_keep(fn_sel_keep_vertices, fn_mapping_body2head))
// TEMPORARILY TURNED OFF            {
// TEMPORARILY TURNED OFF                std::cerr << "Pipeline: Unable to load vertices to keep. Filename: \"" << fn_sel_keep_vertices << "\"" << std::endl;
// TEMPORARILY TURNED OFF                return false;
// TEMPORARILY TURNED OFF            }
// TEMPORARILY TURNED OFF        }

// TEMPORARILY TURNED OFF        Energy_term* E_keep = E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES);
// TEMPORARILY TURNED OFF        E_keep->set_weight(cfg->head().dbl_keep_vertices_weight);
// TEMPORARILY TURNED OFF    }

    // load vertices to omit during fitting
// TEMPORARILY TURNED OFF    std::string fn_sel_vertices_to_omit = cfg->tmpl().fn_sel_vertices_to_omit;
// TEMPORARILY TURNED OFF    if (fn_mapping_body2head.empty())
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        if (!template_fit.load_selection_to_omit(fn_sel_vertices_to_omit))
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            std::cerr << "Pipeline: Unable to load vertices to omit. Filename: \"" << fn_sel_vertices_to_omit << "\"" << std::endl;
// TEMPORARILY TURNED OFF            return false;
// TEMPORARILY TURNED OFF        }
// TEMPORARILY TURNED OFF    }
// TEMPORARILY TURNED OFF    else
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        if (!template_fit.load_selection_to_omit(fn_sel_vertices_to_omit, fn_mapping_body2head))
// TEMPORARILY TURNED OFF        {
// TEMPORARILY TURNED OFF            std::cerr << "Pipeline: Unable to load vertices to omit. Filename: \"" << fn_sel_vertices_to_omit << "\"" << std::endl;
// TEMPORARILY TURNED OFF            return false;
// TEMPORARILY TURNED OFF        }
// TEMPORARILY TURNED OFF    }

    //setup remaining energies
    surface_mesh::Energy_term* E_lm   = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);
    E_lm->set_weight(1.0);
    E_lm->set_multiply_factor(0.1); // 0.5

    surface_mesh::Energy_term* E_cpc  = E.energy_term_list().get_term(surface_mesh::ET_POINT_TO_POINT);
    E_cpc->set_weight(1.0);
    E_cpc->set_multiply_factor(1.0);

    surface_mesh::Energy_term* E_reg  = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_LAPLACIAN_COORDINATES);
    E_reg->set_weight(1.0);
    E_reg->set_multiply_factor(0.1);

    // set lambda termination criteria
    E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_LAPLACIAN_COORDINATES);


    std::cout << std::endl << "Pipeline: [STATUS] Non-rigid head fitting to CPC and landmarks (coarse)." << std::endl;

    template_fit.start(corr_settings, 200, 0.05, 1e-7, true, false);

    template_model_->update_mesh();
    window_.reset_view();
    window_.update();

    window_.print_message_and_pause("AFTER COARSE FIT");

    if (cfg->head().int_detail_level >= 1)
    {
        //detailed shape fitting
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid fitting to CPC and landmarks (fine)." << std::endl;

        corr_settings.dist_until   = 0.02;
        corr_settings.normal_until = 30.0;
        template_fit.start(corr_settings, 200, 0.05, 1e-9, true, false);

        template_model_->update_mesh();
        window_.reset_view();
        window_.update();
    }

//    window_.print_message_and_pause("AFTER FINE FIT");

    if (cfg->head().int_detail_level >= 2)
    {
        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid fitting to CPC and landmarks (extra fine)." << std::endl;

        //change correspondence settings
        corr_settings.dist_until   = 0.01; // 0.005
        corr_settings.normal_until = 15.0; // 20.0

        template_fit.start(corr_settings, 200, 0.05, 1e-10, true, false);

        template_model_->update_mesh();
        pointset_head_->set_visible(false);
        window_.update();
    }

//    window_.print_message_and_pause("AFTER EXTRA FIT");

// TEMPORARILY TURNED OFF    if (! fn_vw_face_and_ears_fitting.empty())
// TEMPORARILY TURNED OFF    {
// TEMPORARILY TURNED OFF        //clear vertex weighting to make sure it does not interfere
// TEMPORARILY TURNED OFF        //with further fitting steps
// TEMPORARILY TURNED OFF        template_fit.clear_vertexweighting("v:fitting_weight");
// TEMPORARILY TURNED OFF    }

    return true;
}

}
