#include "characterfittingwrapper.h"
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <ostream>
#include <fstream>
#include <src/graphene/surface_mesh/algorithms/templatefitting/Landmarks_manager.h>
#include <src/graphene/surface_mesh/algorithms/templatefitting/Template_fit.h>
#include <src/graphene/pointset_processing/Pointset_processing.h>
#include <src/graphene/surface_mesh/algorithms/headtemplatefittingtools/facialfeatures/Facial_features_detection.h>


CharacterFittingWrapper::CharacterFittingWrapper()
{
    template_mesh = nullptr;
    fragment_mesh = nullptr;
    template_model_ = new scene_graph::Surface_mesh_node();
    pointset_fragment_ = new scene_graph::Point_set_node();
}


bool CharacterFittingWrapper::run()
{
    if(setup()){

        std::string fn_result = "./NonRigidRegistration/results/result.obj";
        system("cd NonRigidRegistration\n ls\n ./start_head_fitting.sh /media/andreas/Volume/Progetti/andreaslib/build-debug/NonRigidRegistration");
        ExtendedTrimesh* result = new ExtendedTrimesh();
        result->loadOBJ(fn_result.c_str());
        IMATI_STL::Node* n1 = result->V.head();
        IMATI_STL::Node* n2 = template_mesh->V.head();
        for(unsigned int i = 0; i < result->V.numels(); i++){
            static_cast<IMATI_STL::Vertex*>(n2->data)->setValue(static_cast<IMATI_STL::Vertex*>(n1->data));
            n1 = n1->next();
            n2 = n2->next();
        }
        return true;
    }

//    if(setup()){
//        surface_mesh::Template_fit template_fit;
//        template_fit.set_point_set(&pointset_fragment_->point_set());
//        template_fit.set_template_mesh(&template_model_->mesh());

//        surface_mesh::Energy& E = template_fit.energy();
//        E.energy_term_list().get_term(surface_mesh::ET_MOUTH_SHUT)->set_weight(0);
//        E.energy_term_list().get_term(surface_mesh::ET_EARS_LM)->set_weight(0);
//        E.energy_term_list().get_term(surface_mesh::ET_KEEP_VERTICES)->set_weight(0);
//        surface_mesh::Correspondences_settings corr_settings;
//        corr_settings.filter_dist     = true;
//        corr_settings.dist_until      = 0.2; // 0.02 / 0.01
//        corr_settings.filter_normals  = true;
//        corr_settings.normal_until    = 35.0; // 45.0 / 20.0
//        corr_settings.constr_dir      = surface_mesh::corresp_dir_ps2mesh;

//        surface_mesh::Energy_term* E_lm = E.energy_term_list().get_term(surface_mesh::ET_LANDMARKS);
//        E_lm->set_weight(1.0);
//        E_lm->set_multiply_factor(0.1); // 0.5

//        surface_mesh::Energy_term* E_cpc  = E.energy_term_list().get_term(surface_mesh::ET_POINT_TO_POINT);
//        E_cpc->set_weight(1.0);
//        E_cpc->set_multiply_factor(1.0);

//        surface_mesh::Energy_term* E_reg  = E.energy_term_list().get_term(surface_mesh::ET_NONLIN_LAPLACIAN_COORDINATES);
//        E_reg->set_weight(1.0);
//        E_reg->set_multiply_factor(0.1);

//        // set lambda termination criteria
//        E.energy_term_list().set_term_for_termination_lambda(surface_mesh::ET_NONLIN_LAPLACIAN_COORDINATES);
//        std::cout << std::endl << "Pipeline: [STATUS] Non-rigid head fitting to CPC and landmarks (coarse)." << std::endl;

//        template_fit.start(corr_settings, 200, 0.05, 1e-7, true, false);

//        template_model_->update_mesh();
//        return true;
//    }else
//        return false;
}

bool CharacterFittingWrapper::setup()
{
    mkdir("./NonRigidRegistration/Shapes", 0777);
    mkdir("./NonRigidRegistration/Landmarks", 0777);
    std::string fn_template = "./NonRigidRegistration/Shapes/template_mesh.obj";
    std::string fn_fragment = "./NonRigidRegistration/Shapes/fragment_mesh.xyz";
    template_mesh->saveOBJ(fn_template.c_str());
    fragment_mesh->saveAnnotationsAsXYZ(fn_fragment.c_str());

    if(!save_correspondences_indices()){
        std::cerr << "Pipeline: [ERROR] Could not save correspondences." << std::endl;
        // abort and go into main window loop
        return false;
    }
    if (!template_model_->load(fn_template))
    {
        std::cerr << "Pipeline: [ERROR] Could not load template mesh." << std::endl;
        // abort and go into main window loop
        return false;
    }
    if (!pointset_fragment_->load(fn_fragment))
    {
        std::cerr << "Pipeline: [ERROR] Could not load fragment mesh." << std::endl;
        // abort and go into main window loop
        return false;
    }

    graphene::surface_mesh::Landmarks_manager lmm;
    std::cout << "First load: " << lmm.load(&template_model_->mesh(), "template_landmarks.sel") << std::endl;
    std::cout << "Second load: " << lmm.load(&pointset_fragment_->point_set(), "fragment_landmarks.sel") << std::endl;

    return true;
//    mkdir("./NonRigidRegistration/Shapes", 0777);
//    mkdir("./NonRigidRegistration/Landmarks", 0777);
//    std::string fn_template = "./NonRigidRegistration/Shapes/template_mesh.obj";
//    std::string fn_fragment = "./NonRigidRegistration/Shapes/fragment_mesh.xyz";
//    template_mesh->saveOBJ(fn_template.c_str());
//    fragment_mesh->saveXYZ(fn_fragment.c_str());

//    if(!save_correspondences_indices()){
//        std::cerr << "Pipeline: [ERROR] Could not save correspondences." << std::endl;
//        // abort and go into main window loop
//        return false;
//    }
//    if (!template_model_->load(fn_template))
//    {
//        std::cerr << "Pipeline: [ERROR] Could not load template mesh." << std::endl;
//        // abort and go into main window loop
//        return false;
//    }
//    if (!pointset_fragment_->load(fn_fragment))
//    {
//        std::cerr << "Pipeline: [ERROR] Could not load fragment mesh." << std::endl;
//        // abort and go into main window loop
//        return false;
//    }

//    graphene::surface_mesh::Landmarks_manager lmm;
//    std::cout << "First load: " << lmm.load(&template_model_->mesh(), "template_landmarks.sel") << std::endl;
//    std::cout << "Second load: " << lmm.load(&pointset_fragment_->point_set(), "fragment_landmarks.sel") << std::endl;

//    return true;

}

bool CharacterFittingWrapper::save_correspondences_indices()
{
    std::ofstream template_landmarks_file, fragment_landmarks_file;
    template_landmarks_file.open("./NonRigidRegistration/Landmarks/template_landmarks.sel");
    fragment_landmarks_file.open("./NonRigidRegistration/Landmarks/fragment_landmarks.sel");
    if(template_landmarks_file.is_open() && fragment_landmarks_file.is_open()){
        for(unsigned int i = 0; i < correspondences_indices.size(); i++){
            template_landmarks_file << correspondences_indices[i].first << std::endl;
            fragment_landmarks_file << correspondences_indices[i].second << std::endl;
        }
        template_landmarks_file.close();
        fragment_landmarks_file.close();
        return true;
    }

    std::cerr << "Error while saving correspondences" << std::endl;
    return false;

}

ExtendedTrimesh *CharacterFittingWrapper::getTemplate_mesh() const
{
    return template_mesh;
}

void CharacterFittingWrapper::setTemplate_mesh(ExtendedTrimesh *value)
{
    template_mesh = value;
}

ExtendedTrimesh *CharacterFittingWrapper::getFragment_mesh() const
{
    return fragment_mesh;
}

void CharacterFittingWrapper::setFragment_mesh(ExtendedTrimesh *value)
{
    fragment_mesh = value;
}

std::vector<std::pair<int, int> > CharacterFittingWrapper::getCorrespondences_indices() const
{
    return correspondences_indices;
}

void CharacterFittingWrapper::setCorrespondences_indices(const std::vector<std::pair<int, int> > &value)
{
    correspondences_indices = value;
}
