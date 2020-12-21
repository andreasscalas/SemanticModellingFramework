//=============================================================================
#ifndef GRAPHENE_TEXTURE_PROCESSING_H
#define GRAPHENE_TEXTURE_PROCESSING_H
//=============================================================================

//== INCLUDES =================================================================

#include <array>

#include <opencv2/opencv.hpp>

#include <graphene/surface_mesh/scene_graph/Surface_mesh_node.h>
#include <graphene/character/scene_graph/Character_node.h>

#include <graphene/surface_mesh/algorithms/texture_processing/utility/Color_warp_rbf.h>


//== NAMESPACE ================================================================


namespace graphene {
namespace utility {


//== CLASS DEFINITION =========================================================


class Texture_processing
{

public:


    // constructor
    Texture_processing();

    bool compute_texture_photoscan(const std::string& photoscan_exec,
            const std::string& scancap_dir,
            const std::string& fn_photoscan_project,
            const std::string& fn_invtrans_mesh,
            const std::string& fn_photoscan_texture,
            const int texture_resolution = 4096);

    bool texturemerge(gl::Texture_type dst_texturetype,
                      const std::string& src_texturefilename,
                      const std::string& mask_filename,
                      const std::string &dirichlet_mask_filename,
                      bool use_poisson,
                      scene_graph::Surface_mesh_node& mesh_node);

    bool repair_hand_texture(const std::string &dir_template_db,
                             const std::string& fn_mask_hands_comparison,
                             const std::string& fn_mask_hands_repair,
                             const std::string &fn_mask_hands_dirichlet,
                             scene_graph::Surface_mesh_node& mesh_node);

    bool merge_hand_texture(const std::string& fn_hand_texture,
                            const std::string& fn_hand_mask,
                            const std::string &fn_hand_mask_dirichlet,
                            scene_graph::Surface_mesh_node& mesh_node);

    bool repair_armpit_texture(
            const std::string &fn_mask_armpit_repair,
            scene_graph::Surface_mesh_node &mesh_node);

    bool adjust_eye_and_teeth_luminance(
            const std::string &fn_mask_face_and_ears,
            const std::string &fn_mask_eyes_and_teeth_inverse,
            bool invert_target_mask,
            bool invert_direction,
            scene_graph::Surface_mesh_node& mesh_node
            );

    bool adjust_iris_color(scene_graph::Character_node& mesh_node, const std::string& fn_frontal_image, const std::string& fn_contour_eyes);


public:


    // ... binary mask in [0 , 255] with one channel ...
    bool gaussian_blur(cv::Mat& image, const cv::Mat& image_mask, const unsigned int size, bool& resized) const;

    // ... continuous mask in [0 , 255] with one channel ...
    bool contrast_brightness(cv::Mat& image, const cv::Mat& image_mask, const double contrast_parameter, const double brightness_parameter, bool& resized) const;

    bool kmeans(cv::Mat& image, const int cluster_count) const;

    bool linear_blend(cv::Mat& image, const cv::Mat& image_A, const cv::Mat& image_B, const cv::Mat& image_mask, const double val, bool& resized) const;

    bool copy_and_paste(cv::Mat& image, const cv::Mat& image_src, bool& resized) const;
    bool copy_and_paste(cv::Mat& image, const cv::Mat& image_src, const cv::Mat& image_mask, bool& resized) const;

    bool seamless_clone(cv::Mat& image, const cv::Mat& image_src, const cv::Mat& image_mask, bool& resized,
                        const bool use_opencv_PIE = false, const cv::Mat& dirichlet_mask = cv::Mat(),
                        const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood = std::vector< std::vector< std::vector< std::array<int, 2> > > >()) const;

    bool luminance_mean_shift(const cv::Mat& source_image, cv::Mat& target_image,
                              const cv::Mat& source_mask, const cv::Mat& target_mask,
                              const cv::Mat& reference_mask, bool& resized,
                              const bool invert_direction = false) const;

    bool luminance_mean_shift(const unsigned int mean_luminance, cv::Mat& target_image,
                              const cv::Mat& target_mask, const cv::Mat& reference_mask,
                              bool& resized) const;

    bool color_warp_rbf(const std::vector<Color_warp_rbf::Color_constraint>& color_constraints_lab, cv::Mat& target_image, const cv::Mat& target_mask, bool& resized) const;

    bool generate_mask(const int dim_texture, graphene::surface_mesh::Surface_mesh& mesh, cv::Mat& image_mask) const;

    bool generate_selection(graphene::surface_mesh::Surface_mesh& mesh, const cv::Mat& image_mask) const;

    bool compute_mask_from_black_areas(cv::Mat& out_mask, const cv::Mat& image, const cv::Mat& source_mask) const;

private:

    //helpers
    bool round_to_next_valid_pixel(const double pix_i, const double pix_j, const cv::Mat& texture_colored, graphene::Vec2ui& result) const;

    //Histogram Matching Helpers
    int imhist(cv::Mat image, int histogram[], int channel);
    void extract_iris(cv::Mat& eye_image, int& iris_radius);
    void extract_eye(cv::Mat& image, cv::Mat& eye_image, int radius, Vec2i center);

    //
    // TODO(swenninger): cleanup
    //
    void extract_iris_template(cv::Mat& eye_image, int iris_radius);
    void extract_eye_template(cv::Mat& image, cv::Mat& eye_image, int radius, Vec2i center);


    void filter_skin(cv::Mat& image);
    void histDisplay(int histogram[], const char* name);

};


//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
#endif // GRAPHENE_TEXTURE_PROCESSING_H
//=============================================================================
