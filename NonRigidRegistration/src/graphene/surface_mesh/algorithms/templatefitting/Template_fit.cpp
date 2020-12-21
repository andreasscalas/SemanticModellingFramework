//== INCLUDES ===================================================================


#include "Template_fit.h"

#include <limits>
#include <sstream>
#include <fstream>

#include <graphene/surface_mesh/algorithms/surface_mesh_tools/Triangle_kD_tree.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/diffgeo.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>
#include <graphene/geometry/registration.h>
#include <graphene/geometry/bary_coord.h>
#include <graphene/geometry/Matrix3x3.h>
#include <graphene/utility/Stop_watch.h>

#include "settings.h"
#include "utility/my_helper.h"


//== NAMESPACES ================================================================


namespace graphene {
namespace surface_mesh {


using graphene::geometry::Bounding_box;


//== IMPLEMENTATION ============================================================


Template_fit::
Template_fit() :
    point_set_(NULL),
    template_mesh_(NULL)
{
}


//-----------------------------------------------------------------------------


void
Template_fit::
clear()
{
    // invalidate point_set_
    point_set_ = NULL;

    // clean-up mesh properties
    clean_up_mesh_properties();
    // invalidate template_mesh_
    template_mesh_ = NULL;

    energy_.clear();

}


//-----------------------------------------------------------------------------


void
Template_fit::
set_point_set(Point_set* point_set)
{
    //    std::cout << "[INFO] Template_fit::set_point_set. Address: " << point_set << std::endl;


    point_set_ = point_set;
    energy_.energy_term_data().point_set = point_set;
/*
    if (point_set_ != NULL)
    {
        std::cout << "[INFO] Template_fit::set_point_set. #point: " << point_set_->points_.size() << std::endl;
    }
*/
}


//-----------------------------------------------------------------------------


void
Template_fit::
set_template_mesh(Surface_mesh* template_mesh)
{
//    std::cout << "[INFO] Template_fit::set_template_mesh. Address: " << template_mesh << std::endl;



    template_mesh_ = template_mesh;

    energy_.energy_term_data().template_mesh = template_mesh;
/*
    if (template_mesh_ != NULL)
    {
        std::cout << "[INFO] Template_fit::set_template_mesh. #vertices: " << template_mesh_->n_vertices() << std::endl;
    }
*/
//    update_restpose();
    //            energy_.energy_term_data().eyes_manager.set_template_points_eyecontour( *template_mesh );
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_selection_to_omit(const std::string& filename)
{
    std::ifstream ifs(filename.c_str());

    if (ifs.fail())
    {
        std::cerr << "Template_fit::load_selection_to_omit(): [ERROR] Can't open file." << std::endl;
        return false;
    }

    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_selection_to_omit(): [ERROR] No template mesh available." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<bool> template_points_to_omit;
    template_points_to_omit = template_mesh_->vertex_property<bool>("v:omit", false);
    for (auto v : template_mesh_->vertices())
    {
        template_points_to_omit[v] = false;
    }

    while(!ifs.eof())
    {
        std::string line;
        getline(ifs,line);
        std::stringstream sstr(line);
        size_t idx;
        sstr >> idx;
        Surface_mesh::Vertex v = Surface_mesh::Vertex(idx);
        template_points_to_omit[v] = true;
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_selection_to_omit(const std::string& filename, const std::string& filename_mapping)
{
    std::ifstream ifs(filename.c_str());

    if (ifs.fail())
    {
        std::cerr << "Template_fit::load_selection_to_omit(): [ERROR] Can't open file." << std::endl;
        return false;
    }

    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_selection_to_omit(): [ERROR] No template mesh available." << std::endl;
        return false;
    }

    // load mapping
    std::vector<int> mapping_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mapping_vec, filename_mapping))
    {
        std::cerr << "[ERROR] While loading mapping." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<bool> template_points_to_omit;
    template_points_to_omit = template_mesh_->vertex_property<bool>("v:omit", false);
    for (auto v : template_mesh_->vertices())
    {
        template_points_to_omit[v] = false;
    }

    while(!ifs.eof())
    {
        std::string line;
        getline(ifs,line);
        std::stringstream sstr(line);
        size_t idx;
        sstr >> idx;
        const int map = mapping_vec[ idx ];
        if (map == -1)
        {
            std::cerr << "[ERROR] map == -1" << std::endl;
            return false;
        }
        Surface_mesh::Vertex v = Surface_mesh::Vertex(map);
        template_points_to_omit[v] = true;
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


void
Template_fit::
clear_selection_to_omit()
{
    Surface_mesh::Vertex_property<bool> template_points_to_omit;
    template_points_to_omit = template_mesh_->vertex_property<bool>("v:omit", false);

    for (auto v : template_mesh_->vertices())
    {
        template_points_to_omit[v] = false;
    }
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_selection_to_keep(const std::string& filename)
{
    std::ifstream ifs(filename.c_str());

    if (ifs.fail())
    {
        std::cerr << "Template_fit::load_selection_to_keep(): [ERROR] Can't open file \"" << filename << "\"" << std::endl;
        return false;
    }

    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_selection_to_keep(): [ERROR] No template mesh available." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<bool> template_points_to_keep;
    template_points_to_keep = template_mesh_->vertex_property<bool>("v:keep", false);
    for (auto v : template_mesh_->vertices())
    {
        template_points_to_keep[v] = false;
    }

    while(!ifs.eof())
    {
        std::string line;
        getline(ifs,line);
        std::stringstream sstr(line);
        size_t idx;
        sstr >> idx;
        Surface_mesh::Vertex v = Surface_mesh::Vertex(idx);
        template_points_to_keep[v] = true;
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_selection_to_keep(const std::string& filename, const std::string& filename_mapping, const bool add)
{
    std::ifstream ifs(filename.c_str());

    if (ifs.fail())
    {
        std::cerr << "Template_fit::load_selection_to_keep(): [ERROR] Can't open file \"" << filename << "\"" << std::endl;
        return false;
    }

    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_selection_to_keep(): [ERROR] No template mesh available." << std::endl;
        return false;
    }

    // load mapping
    std::vector<int> mapping_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mapping_vec, filename_mapping))
    {
        std::cerr << "Template_fit::load_selection_to_keep(): [ERROR] While loading mapping. Can't open file \"" << filename_mapping << "\"" << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<bool> template_points_to_keep;
    if (add)
    {
        template_points_to_keep = template_mesh_->vertex_property<bool>("v:keep");
    }
    else
    {
        template_points_to_keep = template_mesh_->vertex_property<bool>("v:keep", false);
        for (auto v : template_mesh_->vertices())
        {
            template_points_to_keep[v] = false;
        }
    }

    while(!ifs.eof())
    {
        std::string line;
        getline(ifs,line);
        std::stringstream sstr(line);
        size_t idx;
        sstr >> idx;
        const int map = mapping_vec[ idx ];
        if (map != -1)
        {
            Surface_mesh::Vertex v = Surface_mesh::Vertex(map);
            template_points_to_keep[v] = true;
        }
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


void
Template_fit::
clear_selection_to_keep()
{
    Surface_mesh::Vertex_property<bool> template_points_to_keep;
    template_points_to_keep = template_mesh_->vertex_property<bool>("v:keep", false);

    for (auto v : template_mesh_->vertices())
    {
        template_points_to_keep[v] = false;
    }
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_selection_ground(const std::string& filename)
{
    std::ifstream ifs(filename.c_str());

    if (ifs.fail())
    {
        std::cerr << "Template_fit::load_selection_ground(): [ERROR] Can't open file." << std::endl;
        return false;
    }

    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_selection_ground(): [ERROR] No template mesh available." << std::endl;
        return false;
    }

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > template_points_ground_prop;
    template_points_ground_prop = template_mesh_->mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");
    std::vector<Surface_mesh::Vertex>& template_points_ground = template_points_ground_prop[0];

    template_points_ground.clear();

    while(!ifs.eof())
    {
        std::string line;
        getline(ifs,line);
        std::stringstream sstr(line);
        size_t idx;
        sstr >> idx;
        Surface_mesh::Vertex v = Surface_mesh::Vertex(idx);
        template_points_ground.push_back(v);
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


void
Template_fit::
clear_selection_ground()
{
    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > template_points_ground_prop;
    template_points_ground_prop = template_mesh_->mesh_property< std::vector<Surface_mesh::Vertex> >("m:ground_vertices");

    template_points_ground_prop[0].clear();
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_mouth_shut_selection(const std::string &filename)
{
    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_mouth_shut_selection: [ERROR] No template mesh available." << std::endl;
        return false;
    }


    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh_->mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];

    mouth_shut.clear();

    if (!surface_mesh::load_selection_from_file(mouth_shut, filename))
    {
        std::cerr << "[ERROR] Can't open selection for mouth shut on template: " << filename << std::endl;
        return false;
    }

    if (mouth_shut.size() % 2 != 0)
    {
        std::cerr << "[ERROR] Mouth shut size is not divisible by 2."  << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Template_fit::
load_mouth_shut_selection(const std::string& filename, const std::string& filename_mapping)
{
    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_mouth_shut_selection(): [ERROR] No template mesh available." << std::endl;
        return false;
    }

    // load mapping
    std::vector<int> mapping_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mapping_vec, filename_mapping))
    {
        std::cerr << "Template_fit::load_mouth_shut_selection(): [ERROR] While loading mapping. Can't open file \"" << filename_mapping << "\"" << std::endl;
        return false;
    }

    // load mouth shut selection
    std::vector<Surface_mesh::Vertex> mouth_shut_full;
    if (!graphene::surface_mesh::load_selection_from_file(mouth_shut_full, filename))
    {
        std::cerr << "Template_fit::load_mouth_shut_selection(): [ERROR] Can't open selection for mouth shut on head template. File \"" << filename << "\"" << std::endl;
        return false;
    }

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mouth_shut_prop
        = template_mesh_->mesh_property< std::vector<Surface_mesh::Vertex> >("m:mouth_shut");
    std::vector<Surface_mesh::Vertex>& mouth_shut  = mouth_shut_prop[0];
    mouth_shut.resize(mouth_shut_full.size());
    for (size_t i = 0; i < mouth_shut_full.size(); ++i)
    {
        Surface_mesh::Vertex& full_v = mouth_shut_full[i];

        if (mapping_vec[full_v.idx()] == -1)
        {
            std::cerr << "Template_fit::load_mouth_shut_selection(): [ERROR] mapping_vec[full_v.idx()] == -1" << std::endl;
            return false;
        }

        mouth_shut[i] = Surface_mesh::Vertex(mapping_vec[full_v.idx()]);
    }

    if (mouth_shut.size() % 2 != 0)
    {
        std::cerr << "[ERROR] Mouth shut size is not divisible by 2."  << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------

bool Template_fit::load_ear_landmarks_template(const std::string &filename)
{
    if (template_mesh_ == nullptr)
    {
        std::cerr << "Template_fit::load_ear_landmarks_template: [ERROR] No template mesh available." << std::endl;
        return false;
    }

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mears
            = template_mesh_->mesh_property< std::vector<Surface_mesh::Vertex> >("m:ears_lm");

    std::vector<Surface_mesh::Vertex>& mears_vec = mears[0];

    return load_selection_from_file(mears_vec, filename);
}

//-----------------------------------------------------------------------------

bool
Template_fit::
load_ear_landmarks_template(const std::string& filename, const std::string& filename_mapping)
{
    if (template_mesh_ == nullptr)
    {
        std::cerr << "Template_fit::load_ear_landmarks_template: [ERROR] No template mesh available." << std::endl;
        return false;
    }

    // load mapping
    std::vector<int> mapping_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mapping_vec, filename_mapping))
    {
        std::cerr << "Template_fit::load_ear_landmarks_template(): [ERROR] While loading mapping. Can't open file \"" << filename_mapping << "\"" << std::endl;
        return false;
    }

    // load ear landmarks selection
    std::vector<Surface_mesh::Vertex> ear_landmarks_full;
    if (!graphene::surface_mesh::load_selection_from_file(ear_landmarks_full, filename))
    {
        std::cerr << "Template_fit::load_ear_landmarks_template(): [ERROR] Can't open selection for ear landmarks. File \"" << filename << "\"" << std::endl;
        return false;
    }

    Surface_mesh::Mesh_property< std::vector<Surface_mesh::Vertex> > mears
            = template_mesh_->mesh_property< std::vector<Surface_mesh::Vertex> >("m:ears_lm");
    std::vector<Surface_mesh::Vertex>& mears_vec = mears[0];
    mears_vec.resize(ear_landmarks_full.size());
    for (size_t i = 0; i < ear_landmarks_full.size(); ++i)
    {
        Surface_mesh::Vertex& full_v = ear_landmarks_full[i];

        if (mapping_vec[full_v.idx()] == -1)
        {
            std::cerr << "Template_fit::load_ear_landmarks_template(): [ERROR] mapping_vec[full_v.idx()] == -1" << std::endl;
            return false;
        }

        mears_vec[i] = Surface_mesh::Vertex(mapping_vec[full_v.idx()]);
    }

    return true;
}

//-----------------------------------------------------------------------------

bool Template_fit::load_ear_landmarks_pointset(const std::string &filename)
{
    if (point_set_ == nullptr)
    {
        std::cerr << "Template_fit::load_ear_landmarks_pointset: [ERROR] No pointset available." << std::endl;
        return false;
    }

    return load_selection_from_file(point_set_->lm_ears_, filename);
}

//-----------------------------------------------------------------------------


bool
Template_fit::
load_vertexweighting(const std::string &filename, const std::string& property_name, bool multiply)
{
    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] No template mesh available." << std::endl;
        return false;
    }

    std::ifstream ifs( filename.c_str() );
    if( ifs.is_open() != true )
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] Cannot load weighting from file:\"" << filename << "\"." << std::endl;
        return false;
    }

    //perform sanity check before load
    //count lines and check if #lines == #vertices of mesh
    size_t n_lines = 0;
    std::string line;
    while (ifs)
    {
        std::getline(ifs, line);
        if (line.empty())
            break;

        ++n_lines;
    }

    if (n_lines != template_mesh_->n_vertices())
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] Wrong number of vertexweightings " << n_lines << " vs " << template_mesh_->n_vertices() << "." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<double> property_of_interest = template_mesh_->vertex_property<double>(property_name.c_str(), 1.0);

    //clear and rewind to beginning of file
    ifs.clear();
    ifs.seekg(0);

    double value;

    if (multiply)
    {
        for (auto v : template_mesh_->vertices())
        {
            ifs >> value;
            if (ifs.eof()) break;
            property_of_interest[v] *= value;
        }
    }
    else
    {
        for (auto v : template_mesh_->vertices())
        {
            ifs >> value;
            if (ifs.eof()) break;
            property_of_interest[v] = value;
        }
    }
    return true;
}


//-----------------------------------------------------------------------------

bool
Template_fit::
load_vertexweighting(const std::string& filename, const std::string& filename_mapping, const std::string& property_name, bool multiply)
{
    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] No template mesh available." << std::endl;
        return false;
    }

    // load mapping
    std::vector<int> mapping_vec;
    if (!graphene::surface_mesh::load_selection_from_file(mapping_vec, filename_mapping))
    {
        std::cerr << "[ERROR] While loading mapping." << std::endl;
        return false;
    }

    std::ifstream ifs( filename.c_str() );
    if( ifs.is_open() != true )
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] Cannot load weighting from file:\"" << filename << "\"." << std::endl;
        return false;
    }

    // load values
    std::vector<double> values_vec;
    double value;
    while (true)
    {
        ifs >> value;
        if (!ifs) break;

        values_vec.push_back(value);
    }

    //perform sanity check
    //#valid_mappingss == #vertices of mesh
    size_t n_lines = 0;
    for (unsigned int i = 0; i < mapping_vec.size(); ++i)
    {
        if (mapping_vec[i] == -1)
        {
            continue;
        }      

        ++n_lines;
    }
    if (n_lines != template_mesh_->n_vertices())
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] Wrong number of valid mappings " << n_lines << " vs " << template_mesh_->n_vertices() << "." << std::endl;
        return false;
    }
    if (mapping_vec.size() != values_vec.size())
    {
        std::cerr << "Template_fit::load_vertexweighting: [ERROR] Can't load vertexweighting: mapping_vec.size() != values_vec.size()." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<double> property_of_interest = template_mesh_->vertex_property<double>(property_name.c_str(), 1.0);

    if (multiply)
    {
        for (unsigned int i = 0; i < mapping_vec.size(); ++i)
        {
            const int map = mapping_vec[ i ];

            if (map != -1)
            {
                property_of_interest[Surface_mesh::Vertex(map)] *= values_vec[i];
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < mapping_vec.size(); ++i)
        {
            const int map = mapping_vec[ i ];

            if (map != -1)
            {
                property_of_interest[Surface_mesh::Vertex(map)] = values_vec[i];
            }
        }
    }

    return true;
}

//-----------------------------------------------------------------------------

bool Template_fit::clear_vertexweighting(const std::string &property_name)
{
    if (template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::clear_vertexweighting: [ERROR] No template mesh available." << std::endl;
        return false;
    }

    Surface_mesh::Vertex_property<double> property_of_interest = template_mesh_->get_vertex_property<double>(property_name.c_str());

    if (property_of_interest)
    {
        template_mesh_->remove_vertex_property<double>(property_of_interest);
    }

    return true;

}

//-----------------------------------------------------------------------------


bool
Template_fit::
do_similarity_transformation_lm(const Transformation_direction trans_dir)
{
    if (point_set_ == NULL || template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::do_similarity_transformation_lm(...): [ERROR] Either no template or no point set present. Aborting..." << std::endl;
        return false;
    }

    Landmarks_manager& landmarks_manager = energy_.energy_term_data().landmarks_manager;

    if (! landmarks_manager.check_dimension(point_set_, template_mesh_))
    {
        std::cerr << "Template_fit::do_similarity_transformation_lm(...): [ERROR] Landmarks have different dimensions. Aborting..." << std::endl;
        return false;
    }

    if (trans_dir == trans_dir_mesh2ps)
    {
        std::vector<Point> lm_ps;
        landmarks_manager.get_landmarks_point_set_stdvec(point_set_, lm_ps);
        std::vector<Point> lm_templ;
        landmarks_manager.get_landmarks_template_stdvec(template_mesh_, lm_templ);

        const Mat4f M = geometry::registration(lm_templ,
                                               lm_ps,
                                               graphene::geometry::CONFORMAL_REGISTRATION);
        std::cout << M << std::endl;

        // transform mesh
        transform_mesh(*template_mesh_, M);
        update_restpose();
    }
    else if (trans_dir == trans_dir_ps2mesh)
    {
        std::vector<Point> lm_ps;
        landmarks_manager.get_landmarks_point_set_stdvec(point_set_, lm_ps);
        std::vector<Point> lm_templ;
        landmarks_manager.get_landmarks_template_stdvec(template_mesh_, lm_templ);

        const Mat4f M = geometry::registration(lm_ps,
                                               lm_templ,
                                               graphene::geometry::CONFORMAL_REGISTRATION);

        // transform points
        transform_points(M);
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Template_fit::
do_similarity_transformation_cpc(const Transformation_direction trans_dir,
                                 const Correspondences_settings& constr_settings,
                                 double& relative_change)
{
    if (point_set_ == NULL || template_mesh_ == NULL)
    {
        std::cerr << "Template_fit::do_similarity_transformation_cpc(...): [ERROR] Either no template or no point set present. Aborting..." << std::endl;

        relative_change = 0.0;
        return false;
    }

    if (trans_dir == trans_dir_ps2mesh)
    {
        std::vector<Correspondence> correspondences;
        if (!compute_correspondences(template_mesh_, point_set_, constr_settings, correspondences))
        {
            std::cerr << "Template_fit::do_similarity_transformation_cpc(...): [ERROR] Can't compute correspondences. Aborting..." << std::endl;

            relative_change = 0.0;
            return false;
        }

        std::vector<Point> points_ps;
        std::vector<Point> points_template;
        for (unsigned int i = 0; i < correspondences.size(); ++i)
        {
            points_ps.push_back(correspondences[i].on_ps);
            points_template.push_back(correspondences[i].on_template);
        }

        double temp_relative_change;
        const Mat4f M = geometry::registration(points_ps,
                                               points_template,
                                               graphene::geometry::CONFORMAL_REGISTRATION,
                                               &temp_relative_change);

        // transform points
        transform_points(M);

        relative_change = temp_relative_change;
        return true;
    }
    else
    {
        std::cerr << "[ERROR] in Template_fit::do_similarity_transformation_cpc()" << std::endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------------------------------

void
Template_fit::
start(const Correspondences_settings &constr_settings,
      const unsigned int max_iter,
      const double conv_val,
      const double until_lambda,
      const bool make_plastic,
      const bool no_decrease)
{
    if (template_mesh_ == NULL || point_set_ == NULL)
    {
        std::cerr << "Template_fit::start(): [ERROR] Either no template or no point set present. Aborting..." << std::endl;
        return;
    }

    update_restpose();

    bool must_compute_correspondences  = false;
    double        lambda               = 0.0;
    double        error_total_previous = DBL_MAX;
    double        error_total_current  = 0.0;
    double        conv_val_rel         = 0.0;
    unsigned int  iter_first_loop      = 0;
    unsigned int  iter_second_loop     = 0;
    bool          first_loop_done      = false;
    bool          second_loop_done     = false;

    //TODO Do we need to check valid settings here?!
    //check_if_settings_are_valid( constr_settings, weights_settings, robustness_settings );

    energy_.energy_term_list().update_and_init_active();

    if (energy_.energy_term_list().active_terms().empty())
    {
        std::cerr << "Template_fit::start(): [ERROR] No active energy terms. Aborting..." << std::endl;
        return;
    }

    std::cout << "Template_fit::start: [STATUS] Active energy terms: \n";
    const std::vector<Energy_term*> &active_terms = energy_.energy_term_list().active_terms();
    for (size_t i=0; i < active_terms.size(); ++i)
    {
        const Energy_term* t = active_terms[i];
        std::cout << "  -" << t->get_name() << std::endl;
    }

    must_compute_correspondences = energy_.energy_term_list().need_correspondences();


    while (!first_loop_done)
    {
        iter_first_loop++;
        //std::cout << "1st loop: " << iter_first_loop << std::endl;

        iter_second_loop = 0;
        second_loop_done = false;
        while (!second_loop_done)
        {
            iter_second_loop++;
            //std::cout << "2nd loop: " << iter_second_loop << std::endl;

            if ( must_compute_correspondences )
            {
                //std::cout << "Template_fit::start(): [INFO] Compute correspondences." << std::endl;
                if (!compute_correspondences(template_mesh_, point_set_, constr_settings, energy_.energy_term_data().correspondences))
                    return;
                sum_correspondences(energy_.energy_term_data().correspondences);
            }

            // ------------------------------------------------------------------------------------------- minimize E_total
            // if minimize returns false, a severe error occured -> abort
            if (! energy_.minimize( conv_val ))
            {
                std::cerr << "Template_fit::start(): [ERROR] Minimize failed -> Aborting..." << std::endl;
                return;
            }


            // ------------------------------------------------------------------------------------------- check termination criteria
            error_total_current = energy_.get_function_value();

            conv_val_rel = std::abs(error_total_current - error_total_previous) / error_total_previous;
            if (conv_val_rel < conv_val)
            {
                second_loop_done = true;
            }
            error_total_previous = error_total_current;

            if (iter_second_loop >= max_iter)
            {
                second_loop_done = true;
                first_loop_done = true;
            }
        }

        energy_.energy_term_list().apply_weight_multipliers();
        lambda = energy_.energy_term_list().get_termination_lambda();



        // check termination criteria
        if (lambda < until_lambda || no_decrease)
        {
            first_loop_done = true;
            std::cout << "Template_fit::start: [STATUS] Termination criterion reached." << std::endl;
        }
        else
        {
            const Energy_term* E_lambda = energy_.energy_term_list().get_term_for_termination_lambda();

            std::cout << "Template_fit::start: [STATUS] "
                      << "Iteration " << iter_first_loop << ". "
                      << "Termination criterion: "
                      <<  lambda << " < " << until_lambda
                       << " of "
                       << "\"" << E_lambda->get_name() << "\"."
                       << std::endl;
        }

        // ---------------------------------------------------------------------------------------- allow plastic deformation
        if (make_plastic) update_restpose();
    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
write_mesh_backtransformed(const std::string &filename_mesh )
{
    Surface_mesh tmp_mesh = *template_mesh_;
    auto tmp_mesh_points  = tmp_mesh.vertex_property<Point>("v:point");

    Mat4f inv_trans = Mat4f::identity();
    if (template_mesh_ && point_set_)
    {
        inv_trans = inverse(point_set_->temp_transformation_);
    }

    for (auto v : tmp_mesh.vertices())
    {
        Point p = tmp_mesh_points[v];
        p = affine_transform(inv_trans, p);
        tmp_mesh_points[v] = p;
    }
    tmp_mesh.update_face_normals();
    tmp_mesh.update_vertex_normals();
    tmp_mesh.write(filename_mesh);
}


//-----------------------------------------------------------------------------


double
Template_fit::
evaluate_fitting()
{
    return energy_.evaluate_fitting();
}


//-----------------------------------------------------------------------------

void
Template_fit::
sum_correspondences(const std::vector<Correspondence>& correspondences)
{
    double& sum_constr_weights = energy_.energy_term_data().sum_constr_weights;
    sum_constr_weights = 0.0;
    for (auto c : correspondences)
    {
        sum_constr_weights += c.weight;
    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
update_restpose()
{
#ifdef BE_VERBOSE
    std::cerr << "in: 'update_restpose(...)'" << std::endl;
#endif

    // update normals
    template_mesh_->update_face_normals();
    template_mesh_->update_vertex_normals();

    // clean-up mesh properties
    clean_up_mesh_properties();

    auto template_points = template_mesh_->vertex_property<Point>("v:point");

    // update respose position
    auto template_restpose_points = template_mesh_->vertex_property<Point>("v:rest_pose");
    template_restpose_points.vector() = template_points.vector();

    // update properties
    update_area_property();
    update_cotan_property();
    update_laplaciancoordinate_property();
    update_aniso_laplace_properties();

    // update rotations
    // auto edge_rotation = template_mesh_->edge_property<Mat4f>("e:edge_rotation");
    // for (auto e : template_mesh_->edges())
    // {
    //     edge_rotation[e] = Mat4f::identity();
    // }
    auto localrotPOS  = template_mesh_->vertex_property<Mat4f>("v:localrotationPOS");
    for (auto v : template_mesh_->vertices())
    {
        localrotPOS[v] = Mat4f::identity();
    }
    // update RHS_R{i,e}_xold
    auto template_aniso_laplace = template_mesh_->edge_property<Point>("e:aniso_laplace");
    auto edge_rotation          = template_mesh_->edge_property<Point>("e:edge_rotation");
    for (auto e : template_mesh_->edges())
    {
        edge_rotation[e] = template_aniso_laplace[e];
    }
    auto template_laplacian_coordinates = template_mesh_->vertex_property<Point>("v:laplacian_coordinates");
    auto localrot  = template_mesh_->vertex_property<Point>("v:localrotation");
    for (auto v : template_mesh_->vertices())
    {
        localrot[v] = template_laplacian_coordinates[v];
    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
transform_points(const Mat4f& M)
{
#ifdef BE_VERBOSE
    std::cerr << "in: 'transform_points(...)'" << std::endl;
#endif

    if (!point_set_)
    {
        std::cerr << "Template_fit::transform_points(...): [ERROR] No point set available. Aborting..." << std::endl;
        return;
    }

    // update total transformation of point-set
    Mat4f& trans = point_set_->temp_transformation_;
    trans = M * trans;

    // normal matrix
    Mat3f normal_matrix = transpose(inverse(Mat3f(M)));

    // transform full original pointset if available
    for (unsigned int i = 0; i < point_set_->points_.size(); ++i)
    {
        Point p = point_set_->points_[i];
        p = affine_transform(M, p);
        point_set_->points_[i] = p;

        Point n = point_set_->normals_[i];
        n = normal_matrix * n;
        n.normalize();
        point_set_->normals_[i] = n;
    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
clean_up_mesh_properties()
{
    if (template_mesh_ == NULL)
        return;

    // clean-up mesh properties
    auto template_restpose_points = template_mesh_->get_vertex_property<Point>("v:rest_pose");
    template_mesh_->remove_vertex_property(template_restpose_points);
    auto area_V               = template_mesh_->get_vertex_property<double>("v:area");
    template_mesh_->remove_vertex_property(area_V);
    auto cotan_V              = template_mesh_->get_edge_property<double>("e:cotan");
    template_mesh_->remove_edge_property(cotan_V);
    auto area_E               = template_mesh_->get_edge_property<double>("fitting:area");
    template_mesh_->remove_edge_property(area_E);
    auto cotan_E              = template_mesh_->get_halfedge_property<double>("fitting:cotan");
    template_mesh_->remove_halfedge_property(cotan_E);
    auto inner_edges_prop     = template_mesh_->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");
    template_mesh_->remove_mesh_property(inner_edges_prop);
//    auto edge_rotation        = template_mesh_->edge_property<Mat4f>("e:edge_rotation");
    auto edge_rotation        = template_mesh_->get_edge_property<Point>("e:edge_rotation");
    template_mesh_->remove_edge_property(edge_rotation);
    auto localrotPOS          = template_mesh_->get_vertex_property<Mat4f>("v:localrotationPOS");
    template_mesh_->remove_vertex_property(localrotPOS);
    auto localrot             = template_mesh_->get_vertex_property<Point>("v:localrotation");
    template_mesh_->remove_vertex_property(localrot);
    auto template_laplacian_coordinates = template_mesh_->get_vertex_property<Point>("v:laplacian_coordinates");
    template_mesh_->remove_vertex_property(template_laplacian_coordinates);
    auto template_aniso_laplace = template_mesh_->get_edge_property<Point>("e:aniso_laplace");
    template_mesh_->remove_edge_property(template_aniso_laplace);
}


//-----------------------------------------------------------------------------


void
Template_fit::
update_area_property()
{
    auto area  = template_mesh_->vertex_property<double>("v:area");
    // compute the voronoi area for the vertex
    double& surface_area = energy_.energy_term_data().surface_area;
    surface_area = 0.0;
    for (auto v : template_mesh_->vertices())
    {
        area[v] = voronoi_area(*template_mesh_, v);
        surface_area += area[v];
    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
update_cotan_property()
{
    auto cotan = template_mesh_->edge_property<double>("e:cotan");
    // compute the cot weights for the edges
    for (auto e : template_mesh_->edges())
    {
//        cotan[e] = std::max(0.0, cotan_weight(*template_mesh_, e)); // DANN IST ES NAEHER DRAN AM EDGE-LAPLACE ABER DAS IST GANZ GANZ WICHTIG DA SONST DIE ARAP ENERGIE NEGATIV WERDEN KANN !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ALSO SOOOOOOO MACHEN !!!!!!!!!
        cotan[e] = cotan_weight(*template_mesh_, e); // KLAPPT AUCH / ABER NICHT BEI ORIG STRUCT ENERGIE, DAHER CLAMPE ICH DORT







//        cotan[e] = graphene::surface_mesh::cotan(xp-xq, xr-xq);

    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
update_laplaciancoordinate_property()
{
    auto template_points = template_mesh_->vertex_property<Point>("v:point");
    auto cotan           = template_mesh_->get_edge_property<double>("e:cotan");
    auto area            = template_mesh_->get_vertex_property<double>("v:area");
    if (!cotan || !area)
    {
        std::cerr << "[ERROR] in Template_fit::update_laplaciancoordinate_property()" << std::endl;
        return;
    }

    // compute laplacian coordinates of template vertices
    auto template_laplacian_coordinates = template_mesh_->vertex_property<Point>("v:laplacian_coordinates");
    for (auto v : template_mesh_->vertices())
    {
        Point lapl_coord(0, 0, 0);

        for (auto hc : template_mesh_->halfedges(v))
        {
            const double w    = cotan[template_mesh_->edge(hc)] / (2*area[v]);
            lapl_coord += w * (template_points[template_mesh_->to_vertex(hc)] - template_points[v]);
        }

        template_laplacian_coordinates[v] = lapl_coord;
    }
}


//-----------------------------------------------------------------------------


void
Template_fit::
update_aniso_laplace_properties()
{
    auto cotan            = template_mesh_->halfedge_property<double>("fitting:cotan");
    auto area             = template_mesh_->edge_property<double>("fitting:area");
    auto inner_edges_prop = template_mesh_->get_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");
    if (!inner_edges_prop)
    {
        inner_edges_prop = template_mesh_->add_mesh_property< std::vector< Surface_mesh::Edge > >("inner_edges");
    }
    if (!cotan || !area || !inner_edges_prop)
    {
        std::cerr << "[ERROR] in Template_fit::update_aniso_laplace_properties()" << std::endl;
        return;
    }


    // compute cotan of each triangle corner (identified by halfedge pointing to it)
    for ( auto hit : template_mesh_->halfedges() )
    {
        if ( template_mesh_->is_boundary( hit ) )
        {
            cotan[hit] = 0.0;
        }
        else
        {
            auto vp = template_mesh_->from_vertex( hit );
            auto vq = template_mesh_->to_vertex( hit );
            auto vr = template_mesh_->to_vertex( template_mesh_->next_halfedge( hit ) );

            const Point& xp = template_mesh_->position(vp);
            const Point& xq = template_mesh_->position(vq);
            const Point& xr = template_mesh_->position(vr);

            // const double cot = graphene::surface_mesh::cotan(xp-xq, xr-xq);
            // cotan[hit] = std::max( 0.0, clamp_cot(cot) );
            cotan[hit] = graphene::surface_mesh::cotan(xp-xq, xr-xq);
        }
    }


    // compute area associated to each edge (1/3 of area of incident triangles)
    double &surface_area_aniso = energy_.energy_term_data().surface_area_aniso;
    surface_area_aniso = 0.0;
    for ( auto e : template_mesh_->edges() )
    {
        if ( template_mesh_->is_boundary( e ))
        {
            area[e] = 0.0;
        }
        else
        {
            auto h = template_mesh_->halfedge(e, 0);
            
            auto vp = template_mesh_->from_vertex(h);
            auto vq = template_mesh_->to_vertex(h);
            auto vr = template_mesh_->to_vertex( template_mesh_->next_halfedge(h) );
            auto vs = template_mesh_->to_vertex( template_mesh_->next_halfedge( template_mesh_->opposite_halfedge(h) ) );

            const Point& xp = template_mesh_->position(vp);
            const Point& xq = template_mesh_->position(vq);
            const Point& xr = template_mesh_->position(vr);
            const Point& xs = template_mesh_->position(vs);

            area[e] = (triangle_area(xp,xq,xr) + triangle_area(xp,xq,xs)) / 3.0;
            surface_area_aniso += area[e];
        }
    }


    // compute aniso laplacians (edges)
    auto template_aniso_laplace = template_mesh_->edge_property<Point>("e:aniso_laplace");
    auto template_points        = template_mesh_->vertex_property<Point>("v:point");

    // collect and count inner edges: each one provides a smoothing constraint
    std::vector< Surface_mesh::Edge > inner_edges;
    inner_edges.reserve( template_mesh_->n_edges() );
    for ( auto e : template_mesh_->edges() )
    {
        if ( ! template_mesh_->is_boundary(e) )
        {
            inner_edges.push_back( e );
        }
    }
    inner_edges_prop[0] = inner_edges;

    const unsigned int ne = inner_edges_prop[0].size();
    for ( unsigned int i = 0; i < ne; ++i )
    {
        auto h = template_mesh_->halfedge( inner_edges_prop[0][i], 0 );

        auto vp = template_mesh_->from_vertex( h );
        auto vq = template_mesh_->to_vertex( h );
        auto vr = template_mesh_->to_vertex( template_mesh_->next_halfedge(h) );
        auto vs = template_mesh_->to_vertex( template_mesh_->next_halfedge( template_mesh_->opposite_halfedge(h) ) );

        // cotan values for edge-incident angles
        const double wik = cotan[ h ];
        const double wij = cotan[ template_mesh_->prev_halfedge(h) ];
        const double wim = cotan[ template_mesh_->opposite_halfedge(h) ];
        const double wil = cotan[ template_mesh_->prev_halfedge( template_mesh_->opposite_halfedge(h) ) ];
        const double l_area = area[ inner_edges_prop[0][i] ];

        // weights for edge-based mean curvature vector
        double ws =  (wil + wim) / l_area;
        double wp = -(wik + wil) / l_area;
        double wr =  (wik + wij) / l_area;
        double wq = -(wij + wim) / l_area;


        // compute laplace(current edge)
        Point lapl_xe(0, 0, 0);
        lapl_xe += wq * template_points[vq];
        lapl_xe += ws * template_points[vs];
        lapl_xe += wr * template_points[vr];
        lapl_xe += wp * template_points[vp];

        template_aniso_laplace[ inner_edges_prop[0][i] ] = lapl_xe;
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
