//== INCLUDES ===================================================================


#include "Texture_processing.h"

#include <graphene/surface_mesh/algorithms/texture_processing/utility/Poisson_image_editing.h>
#include <graphene/surface_mesh/algorithms/texture_processing/utility/Channel_mean_shift.h>
#include <graphene/surface_mesh/algorithms/texture_processing/utility/my_helper.h>
#include <graphene/surface_mesh/algorithms/character_generator/Character_generator.h>

#include <graphene/geometry/distance_point_triangle.h>
#include <graphene/geometry/bary_coord.h>

#include <apps/points2character/config/Config.h>


//== NAMESPACES ================================================================


namespace graphene {
namespace utility {


//== IMPLEMENTATION ============================================================


Texture_processing::
Texture_processing()
{}

bool Texture_processing::compute_texture_photoscan(
        const std::string& photoscan_exec,
        const std::string& scancap_dir,
        const std::string& fn_photoscan_project,
        const std::string& fn_invtrans_mesh,
        const std::string& fn_photoscan_texture,
        const int texture_resolution)
{
    std::ifstream f(fn_photoscan_texture);
    if (!f.good())
    {
        if (photoscan_exec.empty())
        {
            std::cerr << "Texture_processing::compute_texture_photoscan: [ERROR] Could not compute texture. PHOTOSCAN_EXEC environment variable not set!" << std::endl;
            return false;
        }

        if (scancap_dir.empty())
        {
            std::cerr << "Texture_processing::compute_texture_photoscan: [ERROR] Could not compute texture. SCANCAP environment variable not set!" << std::endl;
            return false;
        }

        std::string command(photoscan_exec);
        command.append(" -r ");
        command.append(scancap_dir);
        command.append("/texturize.py ");
        command.append(fn_photoscan_project);
        command.append(" ");
        command.append(fn_invtrans_mesh);
        command.append(" ");
        command.append(fn_photoscan_texture);
        command.append(" ");
        command.append(std::to_string(texture_resolution));

        int r = system( command.c_str() );
        if (r != 0)
        {
            std::cerr << "Texture_processing::compute_texture_photoscan: [ERROR] Something went wrong while computing texture for body." << std::endl;
        }
    }
    else
    {
        std::cout << "Texture_processing::compute_texture_photoscan: [STATUS] Texture already computed." << std::endl;
        return true;
    }

    return true;
}

//-----------------------------------------------------------------------------


bool Texture_processing::texturemerge(gl::Texture_type dst_texturetype,
        const std::string &src_texturefilename,
        const std::string &mask_filename,
        const std::string &dirichlet_mask_filename, bool use_poisson,
        scene_graph::Surface_mesh_node &mesh_node)
{

    cv::Mat src_texture = cv::imread(src_texturefilename.c_str());
    if (src_texture.empty())
    {
        std::cerr << "Texture_processing::texturemerge: [ERROR] Could not load source texture." << std::endl;
        return false;
    }


    gl::Texture* dst_texture = mesh_node.get_texture(dst_texturetype);
    if (dst_texture == nullptr)
    {
        std::cerr << "Texture_processing::texturemerge: [ERROR] Destination texture does not exist." << std::endl;
        return false;
    }

    cv::Mat dst_texture_opencv = texture_to_opencv(dst_texture);


    cv::Mat mask;
    if (mask_filename.empty())
    {
        mask = cv::Mat::ones(dst_texture_opencv.size(), 0)*255;
    }
    else
    {
        mask = cv::imread(mask_filename.c_str(), cv::IMREAD_GRAYSCALE);
        if ( mask.empty() )
        {
            mask = cv::Mat::ones(dst_texture_opencv.size(), 0)*255;
            std::cerr << "Texture_processing::texturemerge: [WARNING] Could not load mask. Merging without mask." << std::endl;
        }
    }

    bool resized = false;

    cv::Mat dirichlet_mask;
    if (!dirichlet_mask_filename.empty())
    {
        dirichlet_mask = cv::imread(dirichlet_mask_filename.c_str(), cv::IMREAD_GRAYSCALE);
        if( dirichlet_mask.empty() )
        {
            std::cerr << "Texture_processing::texturemerge: [ERROR] Could not load dirichlet mask." << std::endl;
            return false;
        }
    }

    if (! use_poisson)
    {
        if (! copy_and_paste(dst_texture_opencv, src_texture, mask, resized))
        {
            std::cerr << "Texture_processing::texturemerge: [ERROR] Texture copy and paste failed." << std::endl;
            return false;
        }

    }
    else
    {
        if (! seamless_clone(dst_texture_opencv, src_texture, mask, resized, false, dirichlet_mask))
        {
            std::cerr << "Texture_processing::texturemerge: [ERROR] Texture copy and paste failed." << std::endl;
            return false;
        }
    }

    if (resized)
    {
        const unsigned int new_w = dst_texture_opencv.cols;
        const unsigned int new_h = dst_texture_opencv.rows;
        dst_texture->width_  = new_w;
        dst_texture->height_ = new_h;

        dst_texture->data_.resize(new_w * new_h * 4);
    }

    opencv_to_texture(dst_texture_opencv, dst_texture);

    mesh_node.update_textures();


    return true;
}


//-----------------------------------------------------------------------------


bool Texture_processing::repair_hand_texture(
        const std::string& dir_template_db,
        const std::string& fn_mask_hands_comparison,
        const std::string& fn_mask_hands_repair,
        const std::string& fn_mask_hands_dirichlet,
        scene_graph::Surface_mesh_node& mesh_node)
{


    graphene::gl::Texture* color_texture = nullptr;

    color_texture = mesh_node.get_texture(gl::TT_COLOR);

    if (color_texture == nullptr)
    {
        std::cerr << "Texture_processing::repair_hand_texture: [ERROR] Template has no color texture." << std::endl;
        return false;
    }
    cv::Mat texture_opencv = texture_to_opencv(color_texture);
    const unsigned int dim_texture = texture_opencv.rows;

    cv::Mat image_mask_comparison = cv::imread(fn_mask_hands_comparison.c_str(), cv::IMREAD_GRAYSCALE); // gray-scale mask

    if (image_mask_comparison.empty())
    {
        std::cerr << "Texture_processing::repair_hand_texture: [ERROR] Could not load hand repair comparison mask. Filename: \"" << fn_mask_hands_comparison << "\"." << std::endl;
        return false;
    }
    cv::resize(image_mask_comparison, image_mask_comparison, cv::Size(dim_texture, dim_texture));

    cv::Mat image_mask_hands = cv::imread(fn_mask_hands_repair.c_str(),  cv::IMREAD_GRAYSCALE); // gray-scale mask

    if (image_mask_hands.empty())
    {
        std::cerr << "Texture_processing::repair_hand_texture: [ERROR] Could not load repair mask for hands. Filename: \"" << fn_mask_hands_repair << "\"." << std::endl;
        return false;
    }
    cv::resize(image_mask_hands, image_mask_hands, cv::Size(dim_texture, dim_texture));

    // std::cerr << "[DEBUG] repair hand texture - START!" << std::endl;


    // fit hands (skin)

    graphene::surface_mesh::Character_generator character_generator;
    character_generator.set_dir_database(dir_template_db);

    int min_skin_type_int = graphene::surface_mesh::Skin_type::Skin_M_01;
    double min_diff = DBL_MAX;

    for ( int skin_type_int = graphene::surface_mesh::Skin_type::Skin_M_01;
          skin_type_int < graphene::surface_mesh::Skin_type::Skin_LAST;
          skin_type_int++ )
    {
        /*
        if (is_male)
        {
            // skip W
            if ( skin_type_int >= 42 )
            {
                continue;
            }
        }
        else
        {
            // skip M
            if ( skin_type_int < 42 )
            {
                continue;
            }
        }
        */

        // skip bad hand textures
        if ( skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_04 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_07 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_08 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_11 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_19 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_20 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_22 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_27 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_29 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_39 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_48 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_49 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_07 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_11 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_13 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_14 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_21 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_30 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_39 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_40 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_46 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_47 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_49 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_01 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_03 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_04 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_15 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_28
             )
        {
            continue;
        }

        // skip too bright hand textures
        if ( skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_33 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_09 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_10 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_12 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_18 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_21 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_23 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_24 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_25 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_28 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_30 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_32 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_36 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_37 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_38 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_41 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_44 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_45 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_46 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_M_47 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_08 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_10 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_25 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_26 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_32 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_37 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_05 ||
             skin_type_int == graphene::surface_mesh::Skin_type::Skin_W_38
             )
        {
            continue;
        }

        graphene::surface_mesh::Skin_type current_skin_type = static_cast<graphene::surface_mesh::Skin_type>(skin_type_int);

        const std::string filename_texture_color = character_generator.get_full_filename_for_skintexture_color(current_skin_type);

        cv::Mat cg_texture_color = cv::imread(filename_texture_color.c_str());
        if( cg_texture_color.empty() )
        {
            std::cerr << "Texture_processing::repair_hand_texture: [ERROR] Could not read texture with filename: " << filename_texture_color << std::endl;
            continue;
        }
        cv::resize(cg_texture_color, cg_texture_color, cv::Size(dim_texture, dim_texture));

        double diff = 0.0;
        for (unsigned int i = 0; i < dim_texture; i+=4)
        {
            for (unsigned int j = 0; j < dim_texture; j+=4)
            {
                if ( image_mask_comparison.at<uchar>(i, j) > 0 )
                {
                    const double diff_b = cg_texture_color.at<cv::Vec3b>(i, j)[0] - texture_opencv.at<cv::Vec3b>(i, j)[0];
                    const double diff_g = cg_texture_color.at<cv::Vec3b>(i, j)[1] - texture_opencv.at<cv::Vec3b>(i, j)[1];
                    const double diff_r = cg_texture_color.at<cv::Vec3b>(i, j)[2] - texture_opencv.at<cv::Vec3b>(i, j)[2];

                    diff += sqrt(diff_b*diff_b + diff_g*diff_g + diff_r*diff_r);
                }
            }
        }

        if (diff < min_diff)
        {
            min_skin_type_int = skin_type_int;
            min_diff = diff;
        }
    }



    graphene::surface_mesh::Skin_type best_skin_type = static_cast<graphene::surface_mesh::Skin_type>(min_skin_type_int);

    const std::string filename_texture_best_skin     = character_generator.get_full_filename_for_skintexture_color(best_skin_type);

    std::cout << "Texture_processing::repair_hand_texture: [STATUS] Found best fitting hand skin in texture: \"" << filename_texture_best_skin << "\"." << std::endl;


    cv::Mat cg_texture_best_color = cv::imread(filename_texture_best_skin.c_str());

    cv::resize(cg_texture_best_color, cg_texture_best_color, cv::Size(dim_texture, dim_texture));


    // poisson image editing

    // load dirichlet mask
    cv::Mat dirichlet_mask_hands = cv::imread(fn_mask_hands_dirichlet.c_str(), cv::IMREAD_GRAYSCALE); // gray-scale mask
    bool resized = false;
    if( dirichlet_mask_hands.empty() )
    {
        std::cerr << "Texture_processing::repair_hand_texture: [WARNING] Could not load dirichlet mask for hands. Filename: \"" << fn_mask_hands_dirichlet << "\"." << std::endl;

        if (! seamless_clone(texture_opencv, cg_texture_best_color, image_mask_hands, resized, false))
        {
            std::cerr << "Texture_processing::repair_hand_texture: [ERROR] Seamless clone for hand repair failed."<< std::endl;
            return false;
        }
    }
    else
    {
        cv::resize(dirichlet_mask_hands, dirichlet_mask_hands, cv::Size(dim_texture, dim_texture));

        if (! seamless_clone(texture_opencv, cg_texture_best_color, image_mask_hands, resized, false, dirichlet_mask_hands))
        {
            std::cerr << "Texture_processing::repair_hand_texture: [ERROR] Seamless clone for hand repair failed."<< std::endl;
            return false;
        }
    }

    // eventually resize 'texture_opencv'
    if (resized)
    {
        const unsigned int new_w = texture_opencv.cols;
        const unsigned int new_h = texture_opencv.rows;
        color_texture->width_  = new_w;
        color_texture->height_ = new_h;

        color_texture->data_.resize(new_w * new_h * 4);
    }

    opencv_to_texture(texture_opencv, color_texture);

    mesh_node.update_textures();

    return true;

}


//-----------------------------------------------------------------------------


bool Texture_processing::merge_hand_texture(const std::string& fn_hand_texture,
                                            const std::string& fn_hand_mask,
                                            const std::string &fn_hand_mask_dirichlet,
                                            scene_graph::Surface_mesh_node& mesh_node)
{
    graphene::gl::Texture* color_texture = nullptr;

    color_texture = mesh_node.get_texture(gl::TT_COLOR);

    if (color_texture == nullptr)
    {
        std::cerr << "Texture_processing::merge_hand_texture: [ERROR] Template has no color texture." << std::endl;
        return false;
    }
    cv::Mat texture_opencv = texture_to_opencv(color_texture);

    cv::Mat image_mask = cv::imread(fn_hand_mask.c_str(), cv::IMREAD_GRAYSCALE); // gray-scale mask

    if (image_mask.empty())
    {
        std::cerr << "Texture_processing::merge_hand_texture: [ERROR] Could not load hand mask. Filename: \"" << fn_hand_mask << "\"." << std::endl;
        return false;
    }

    // std::cerr << "[DEBUG] merge hand texture - START!" << std::endl;

    cv::Mat image_hand_texture = cv::imread(fn_hand_texture.c_str());

    if (image_hand_texture.empty())
    {
        std::cerr << "Texture_processing::merge_hand_texture: [ERROR] Could not load hand texture. Filename: \"" << fn_hand_texture << "\"." << std::endl;
        return false;
    }

    const bool ugly_hack_because_wuerzis_hand_scanner_is_so_bad = true;
    if (ugly_hack_because_wuerzis_hand_scanner_is_so_bad)
    {
        for(int k = 0; k < image_hand_texture.rows; ++k)
        {
            for(int j = 0; j < image_hand_texture.cols; ++j)
            {
                image_hand_texture.at<cv::Vec3b>(k, j)[0] /= 1.1; // b
                image_hand_texture.at<cv::Vec3b>(k, j)[1] *= 1.1; // g
                image_hand_texture.at<cv::Vec3b>(k, j)[2] *= 1.1; // r
            }
        }
    }

    // load dirichlet mask
    cv::Mat dirichlet_mask_hands = cv::imread(fn_hand_mask_dirichlet.c_str(), cv::IMREAD_GRAYSCALE); // gray-scale mask
    if( dirichlet_mask_hands.empty() )
    {
        std::cerr << "Texture_processing::merge_hand_texture: [ERROR] Could not load dirichlet mask for hands. Filename: \"" << fn_hand_mask_dirichlet << "\"." << std::endl;
        return false;
    }

    if (texture_opencv.rows       > 6144 ||
        texture_opencv.cols       > 6144 ||
        image_hand_texture.rows   > 6144 ||
        image_hand_texture.cols   > 6144 ||
        image_mask.rows           > 6144 ||
        image_mask.cols           > 6144 ||
        dirichlet_mask_hands.rows > 6144 ||
        dirichlet_mask_hands.cols > 6144)
    {
        std::cerr << "Texture_processing::merge_hand_texture: [WARNING] Merging a hand texture greater than 6K may consume a lot of RAM and takes a while! Check all involved textures and masks! We continue anyway ..." << std::endl;
    }


    // poisson image editing

    bool resized = false;
    if (! seamless_clone(texture_opencv, image_hand_texture, image_mask, resized, false, dirichlet_mask_hands))
    {
        std::cerr << "Texture_processing::merge_hand_texture: [ERROR] Seamless clone for hand repair failed."<< std::endl;
        return false;
    }


    // eventually resize 'texture_opencv'
    if (resized)
    {
        const unsigned int new_w = texture_opencv.cols;
        const unsigned int new_h = texture_opencv.rows;
        color_texture->width_  = new_w;
        color_texture->height_ = new_h;

        color_texture->data_.resize(new_w * new_h * 4);
    }

    opencv_to_texture(texture_opencv, color_texture);

    mesh_node.update_textures();

    return true;
}


//-----------------------------------------------------------------------------


bool Texture_processing::repair_armpit_texture(
        const std::string& fn_mask_armpit_repair,
        scene_graph::Surface_mesh_node& mesh_node
        )
{

    cv::Mat texture_opencv;

    gl::Texture* mesh_color_texture = nullptr;
    mesh_color_texture = mesh_node.get_texture(gl::TT_COLOR);

    if (mesh_color_texture == nullptr)
    {
        std::cerr << "Texture_processing::repair_armpit_texture: [ERROR] Mesh does not have a color texture." << std::endl;
    }

    texture_opencv = texture_to_opencv(mesh_color_texture);

    // mask
    cv::Mat mask;
    mask = cv::imread(fn_mask_armpit_repair.c_str(), cv::IMREAD_GRAYSCALE);

    if( mask.empty() )
    {
        std::cerr << "Texture_processing::repair_armpit_texture: [ERROR] Could not load armpit repair mask with filename: \"" << fn_mask_armpit_repair << "\"." << std::endl;
        return false;
    }

    // eventually resize mask to fit to texture
    if (mask.rows != texture_opencv.rows ||
        mask.cols != texture_opencv.cols)
    {
        if (texture_opencv.rows > 4096 ||
            texture_opencv.cols > 4096)
        {
            std::cerr << "Texture_processing::repair_armpit_texture: [WARNING] Texture is greater than 4K. Repairing armpit may consume a lot of RAM and takes a while! Aborting ..." << std::endl;
            return false;
        }

//        const cv::Size texture_opencv_size(texture_opencv.cols, texture_opencv.rows);
//        cv::resize(mask, mask, texture_opencv_size);
    }

    // for debugging
    // cv::imwrite("DEBUG_MASK.png", mask);

    // label connected components
    cv::Mat labels;
    const int num_CC = cv::connectedComponents(mask,
                                               labels,
                                               4,
                                               CV_16U);
    // std::cerr << "[DEBUG] num_CC: " << num_CC << std::endl;
    const unsigned int rows = labels.rows;
    const unsigned int cols = labels.cols;

    // helper texture, which encodes triangle ids
    cv::Mat texture_colored(rows, cols, CV_8UC3, cv::Scalar(0));
    const surface_mesh::Surface_mesh& mesh = mesh_node.mesh();
    compute_texture_colored_triangle_ids(mesh, texture_colored);

    // for debugging
    // cv::imwrite("DEBUG_TRI_ID.png", texture_colored);

    // halfedges
    auto texcoords = mesh.get_halfedge_property<graphene::Texture_coordinate>("h:texcoord");
    if (!texcoords)
    {
        std::cerr << "Texture_processing::repair_armpit_texture: [ERROR] Mesh has no texture coordinates." << std::endl;
    }

    // compute mapping from component idx to pixel coordinate
    // std::cerr << "[DEBUG] compute mapping from component idx to pixel coordinate" << std::endl;
    std::vector< std::vector<graphene::Vec2ui> > map_ccidx_to_pixels(num_CC);
    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            const unsigned int idx_CC = labels.at<ushort>(i, j);

            const graphene::Vec2ui current_pixel(i, j);
            map_ccidx_to_pixels[idx_CC].push_back(current_pixel);
        }
    }

    // helper, which pixels have jumps
    cv::Mat helper_pixels_jumps = cv::Mat::zeros(rows, cols, CV_8UC1);

    // helper, to store neighboring pixel coordinates (eventually jumping) for each pixel from source mask
    // std::cerr << "[DEBUG] helper, to store neighboring pixel coordinates (eventually jumping) for each pixel from source mask" << std::endl;
    std::vector< std::vector< std::vector< std::array<int, 2> > > > helper_mapping_neighborhood(rows);
    for (unsigned int i = 0; i < helper_mapping_neighborhood.size(); ++i)
    {
        helper_mapping_neighborhood[i].resize(cols);
    }
    // initialize with default neighborhood (l, t, r, b)
    for (int i = 0; i < helper_mapping_neighborhood.size(); ++i)
    {
        for (int j = 0; j < helper_mapping_neighborhood[i].size(); ++j)
        {
            int l = j-1;
            int t = i-1;
            int r = j+1;
            int b = i+1;

            if (l < 0)
            {
                l = 0;
            }
            if (i < 0)
            {
                t = 0;
            }
            if (r > helper_mapping_neighborhood[i].size()-1)
            {
                r = helper_mapping_neighborhood[i].size()-1;
            }
            if (b > helper_mapping_neighborhood.size()-1)
            {
                b = helper_mapping_neighborhood.size()-1;
            }

            helper_mapping_neighborhood[i][j].push_back(std::array<int, 2>{i, l}); // l
            helper_mapping_neighborhood[i][j].push_back(std::array<int, 2>{t, j}); // t
            helper_mapping_neighborhood[i][j].push_back(std::array<int, 2>{i, r}); // r
            helper_mapping_neighborhood[i][j].push_back(std::array<int, 2>{b, j}); // b
        }
    }

    // label (real) connected components (based on 3d neighborhood)
    // std::cerr << "[DEBUG] label (real) connected components (based on 3d neighborhood)" << std::endl;
    cv::Mat labels_real = labels.clone();
    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            // pixel is background, i.e., nothing to do
            if ( labels.at<ushort>(i, j) == 0 )
            {
                continue;
            }

            // check if current connected component is basically connected to a distant connected component

            // first, check if a jump is possible at all, namely if neigboring pixels point to another face id
            const cv::Vec3b pix_current = texture_colored.at<cv::Vec3b>(i, j);
            int faceID_current_pixel = pix_current[2] * 256 * 256 + pix_current[1] * 256 + pix_current[0];
            faceID_current_pixel--; // undo previous increment (-1 means no triangle ID)
            if (faceID_current_pixel < 0)
            {
                continue;
            }
            graphene::surface_mesh::Surface_mesh::Face faceHandle_current_pixel(faceID_current_pixel);
            std::vector< graphene::surface_mesh::Surface_mesh::Face > faceHandles_neighborhood_tex;
            std::vector< std::string > faceHandles_neighborhood_tex_names;
            // left pixel -> face id
            if (j > 0)
            {
                const cv::Vec3b pix_left = texture_colored.at<cv::Vec3b>(i, j-1);
                int faceID_left_pixel = pix_left[2] * 256 * 256 + pix_left[1] * 256 + pix_left[0];
                faceID_left_pixel--; // undo previous increment (-1 means no triangle ID)
                if (faceID_current_pixel != faceID_left_pixel)
                {
                    graphene::surface_mesh::Surface_mesh::Face faceHandle_left_pixel(faceID_left_pixel);
                    faceHandles_neighborhood_tex.push_back(faceHandle_left_pixel);
                    faceHandles_neighborhood_tex_names.push_back("L");
                }
            }
            // right pixel -> face id
            if (j < cols-1)
            {
                const cv::Vec3b pix_right = texture_colored.at<cv::Vec3b>(i, j+1);
                int faceID_right_pixel = pix_right[2] * 256 * 256 + pix_right[1] * 256 + pix_right[0];
                faceID_right_pixel--; // undo previous increment (-1 means no triangle ID)
                if (faceID_current_pixel != faceID_right_pixel)
                {
                    graphene::surface_mesh::Surface_mesh::Face faceHandle_right_pixel(faceID_right_pixel);
                    faceHandles_neighborhood_tex.push_back(faceHandle_right_pixel);
                    faceHandles_neighborhood_tex_names.push_back("R");
                }
            }
            // top pixel -> face id
            if (i > 0)
            {
                const cv::Vec3b pix_top = texture_colored.at<cv::Vec3b>(i-1, j);
                int faceID_top_pixel = pix_top[2] * 256 * 256 + pix_top[1] * 256 + pix_top[0];
                faceID_top_pixel--; // undo previous increment (-1 means no triangle ID)
                if (faceID_current_pixel != faceID_top_pixel)
                {
                    graphene::surface_mesh::Surface_mesh::Face faceHandle_top_pixel(faceID_top_pixel);
                    faceHandles_neighborhood_tex.push_back(faceHandle_top_pixel);
                    faceHandles_neighborhood_tex_names.push_back("T");
                }
            }
            // bottom pixel -> face id
            if (i < rows-1)
            {
                const cv::Vec3b pix_bottom = texture_colored.at<cv::Vec3b>(i+1, j);
                int faceID_bottom_pixel = pix_bottom[2] * 256 * 256 + pix_bottom[1] * 256 + pix_bottom[0];
                faceID_bottom_pixel--; // undo previous increment (-1 means no triangle ID)
                if (faceID_current_pixel != faceID_bottom_pixel)
                {
                    graphene::surface_mesh::Surface_mesh::Face faceHandle_bottom_pixel(faceID_bottom_pixel);
                    faceHandles_neighborhood_tex.push_back(faceHandle_bottom_pixel);
                    faceHandles_neighborhood_tex_names.push_back("B");
                }
            }

            // if no jump is possible, do nothing, i.e., do not change any labeling as we cannot be connected to a distant component and do not update any mapping for neighbors
            if (faceHandles_neighborhood_tex.size() == 0)
            {
                continue;
            }

            // if a jump is possible, check if we are basically connected to another distant connected component

            // find candidates (real 3d neighbors)

            std::set< graphene::surface_mesh::Surface_mesh::Face > face_candidates_real_neighbors;
            for (auto fv : mesh.vertices(faceHandle_current_pixel))
            {
                for (auto vf : mesh.faces(fv))
                {
                    face_candidates_real_neighbors.insert(vf);
                }
            }
            face_candidates_real_neighbors.erase(faceHandle_current_pixel);

            // compute current 3d point on current triangle
            graphene::Point current_3d_point(0);
            {
                Eigen::Matrix3d U;
                int index = 0; // column
                for (auto fh : mesh.halfedges(faceHandle_current_pixel))
                {
                    graphene::Texture_coordinate t = texcoords[fh];
                    t[2] = 1.0;
                    for (unsigned int i = 0; i < 3; ++i)
                    {
                        U(i, index) = t[i];
                    }
                    index++;
                }

                Eigen::Vector3d rhs(       static_cast<double>(j) / (static_cast<double>(cols) - 1.0),
                                           1.0 - (static_cast<double>(i) / (static_cast<double>(rows) - 1.0)),
                                           1.0);
                Eigen::Vector3d A = U.inverse() * rhs;

                index = 0;
                for (auto fh : mesh.halfedges(faceHandle_current_pixel))
                {
                    current_3d_point += A(index)*mesh.position(mesh.to_vertex(fh));
                    index++;
                }
            }

            // iterate pixel neighborhood
            for (unsigned int n = 0; n < faceHandles_neighborhood_tex.size(); ++n)
            {
                const graphene::surface_mesh::Surface_mesh::Face& current_neighbor_tex = faceHandles_neighborhood_tex[n];
                const std::string& current_neighbor_tex_name = faceHandles_neighborhood_tex_names[n];
                // check if this neighboring handle (in texture) is a candidate (in 3d)
                if ( std::find(face_candidates_real_neighbors.begin(), face_candidates_real_neighbors.end(), current_neighbor_tex) !=
                     face_candidates_real_neighbors.end() )
                {
                    // 'face_candidates_real_neighbors' contains 'current_neighbor_tex', i.e., there is no jump and nothing to merge
                    continue;
                }
                else
                {
                    // there is a jump
                    helper_pixels_jumps.at<uchar>(i, j) = 255;

                    // there is a jump, i.e., find (real) neighboring triangle next to current 3d point
                    graphene::surface_mesh::Surface_mesh::Face next_triangle;
                    double min_dist = DBL_MAX;
                    Point total_nearest_point(0);
                    // iterate through real neighboring triangles
                    // find triangle next to current 3d point
                    // find closest point on that triangle
                    for (auto real_neighbor : face_candidates_real_neighbors)
                    {
                        // get triangle vertices of real neighboring triangle
                        auto fvit = mesh.vertices(real_neighbor);
                        const auto   v0 = *fvit;
                        const auto   v1 = *(++fvit);
                        const auto   v2 = *(++fvit);

                        const graphene::Point p0 = mesh.position(v0);
                        const graphene::Point p1 = mesh.position(v1);
                        const graphene::Point p2 = mesh.position(v2);

                        Point nearest_point;
                        const double dist = graphene::geometry::dist_point_triangle(current_3d_point,
                                                                                    p0, p1, p2,
                                                                                    nearest_point);
                        if (dist < min_dist)
                        {
                            next_triangle = real_neighbor;
                            min_dist = dist;
                            total_nearest_point = nearest_point;
                        }
                    }

                    // compute barycoords w.r.t. next triangle
                    auto fvit = mesh.vertices(next_triangle);
                    const auto   v0 = *fvit;
                    const auto   v1 = *(++fvit);
                    const auto   v2 = *(++fvit);
                    const graphene::Point p0 = mesh.position(v0);
                    const graphene::Point p1 = mesh.position(v1);
                    const graphene::Point p2 = mesh.position(v2);
                    graphene::Point b = graphene::geometry::barycentric_coordinates(total_nearest_point, p0, p1, p2);
                    if (b[0] < -0.0001 || b[0] > 1.0001 || b[1] < -0.0001 || b[1] > 1.0001 || b[2] < -0.0001 || b[2] > 1.0001)
                    {
                        std::cerr << "[ERROR] SHOULD NOT HAPPEN !!!" << std::endl;
                        continue;
                    }
                    if (b[0] < 0.0) b[0] = 0.0;
                    if (b[0] > 1.0) b[0] = 1.0;
                    if (b[1] < 0.0) b[1] = 0.0;
                    if (b[1] > 1.0) b[1] = 1.0;
                    if (b[2] < 0.0) b[2] = 0.0;
                    if (b[2] > 1.0) b[2] = 1.0;

                    // access texture
                    double interp_tc_i = 0;
                    double interp_tc_j = 0;
                    int index = 0;
                    for (auto fh : mesh.halfedges(next_triangle))
                    {
                        const graphene::Texture_coordinate t = texcoords[fh];

                        interp_tc_j += b[index]*t[0];
                        interp_tc_i += b[index]*t[1];

                        index++;
                    }
                    interp_tc_i = 1.0 - interp_tc_i;
                    interp_tc_i *= static_cast<double>(rows) - 1.0;
                    interp_tc_j *= static_cast<double>(cols) - 1.0;
                    graphene::Vec2ui interp_pixel;
                    if (!round_to_next_valid_pixel(interp_tc_i, interp_tc_j, texture_colored, interp_pixel))
                    {
                        std::cerr << "[ERROR] CANNOT FIND VALID PIXEL! SHOULD NOT HAPPEN !!!" << std::endl;
                        continue;
                    }
                    const int interp_pixel_x = interp_pixel[1];
                    const int interp_pixel_y = interp_pixel[0];

                    // check label of that connected component
                    const unsigned int label_distant_CC = labels.at<ushort>(interp_pixel_y, interp_pixel_x);

                    // change label of that connected component to current label, i.e., merge
                    const unsigned int label_current_CC = labels_real.at<ushort>(i, j);

                    // update neighborhood mapping
                    if ( current_neighbor_tex_name == "L" )
                    {
                        helper_mapping_neighborhood[i][j][0] = std::array<int, 2>{interp_pixel_y, interp_pixel_x};
                    }
                    else if ( current_neighbor_tex_name == "T" )
                    {
                        helper_mapping_neighborhood[i][j][1] = std::array<int, 2>{interp_pixel_y, interp_pixel_x};
                    }
                    else if ( current_neighbor_tex_name == "R" )
                    {
                        helper_mapping_neighborhood[i][j][2] = std::array<int, 2>{interp_pixel_y, interp_pixel_x};
                    }
                    else if ( current_neighbor_tex_name == "B" )
                    {
                        helper_mapping_neighborhood[i][j][3] = std::array<int, 2>{interp_pixel_y, interp_pixel_x};
                    }
                    // if current label id is equal to distant label id, we have nothing to merge
                    if ( (label_distant_CC == 0) || (label_current_CC == label_distant_CC) )
                    {
                        continue;
                    }
                    // update labels
                    for (unsigned int remote_CC_idx = 0; remote_CC_idx < map_ccidx_to_pixels[label_distant_CC].size(); ++remote_CC_idx)
                    {
                        const unsigned int target_i = map_ccidx_to_pixels[label_distant_CC][remote_CC_idx][0];
                        const unsigned int target_j = map_ccidx_to_pixels[label_distant_CC][remote_CC_idx][1];

                        labels_real.at<ushort>(target_i, target_j) = label_current_CC;
                    }
                }
            }
        }
    }

    // re-label
    // std::cerr << "[DEBUG] re-label" << std::endl;
    int max_label = 0;
    bool done = false;
    do
    {
        done = false;

        // find max label id
        max_label = 0;
        for (unsigned int i = 0; i < rows; ++i)
        {
            for (unsigned int j = 0; j < cols; ++j)
            {
                if ( labels_real.at<ushort>(i, j) == 0 )
                {
                    continue;
                }

                const unsigned int current_id = labels_real.at<ushort>(i, j);
                if ( current_id > max_label )
                {
                    max_label = current_id;
                }
            }
        }

        if (max_label == 0)
        {
            done = true;
            break;
        }

        // find free label
        int free_label = 1;
        bool found_free_label = true;
        bool double_break = false;
        while (free_label != max_label)
        {
            found_free_label = true;
            for (unsigned int i = 0; i < rows; ++i)
            {
                for (unsigned int j = 0; j < cols; ++j)
                {
                    if ( labels_real.at<ushort>(i, j) == free_label )
                    {
                        free_label++;
                        found_free_label = false;
                        double_break = true;
                        break;
                    }
                }
                if (double_break)
                {
                    break;
                }
            }
            if (found_free_label)
            {
                break;
            }
        }
        if (free_label == max_label)
        {
            found_free_label = false;
        }

        if (found_free_label) // rename max label to free label
        {
            for (unsigned int i = 0; i < rows; ++i)
            {
                for (unsigned int j = 0; j < cols; ++j)
                {
                    if ( labels_real.at<ushort>(i, j) == max_label )
                    {
                        labels_real.at<ushort>(i, j) = free_label;
                    }
                }
            }
        }
        else // otherwise we can not swap
        {
            done = true;
        }
    }
    while(!done);


    for (unsigned int cc = 1; cc <= max_label; ++cc)
    {
        const cv::Mat src = cv::Mat::zeros(rows, cols, CV_8UC3); // zero gradients

        // compute mask for current connected component (must be gray-level)
        cv::Mat current_mask_CC = cv::Mat::zeros(rows, cols, 0);
        for (unsigned int i = 0; i < rows; ++i)
        {
            for (unsigned int j = 0; j < cols; ++j)
            {
                if ( labels_real.at<ushort>(i, j) == cc )
                {
                    current_mask_CC.at<uchar>(i, j) = 255;
                }
            }
        }

        // prepare dirichlet mask
        cv::Mat helper_dirichlet_mask = cv::Mat::zeros(rows, cols, CV_8UC1);
        for (unsigned int i = 0; i < rows; ++i)
        {
            for (unsigned int j = 0; j < cols; ++j)
            {
                if (helper_dirichlet_mask.at<uchar>(i, j) == 255)
                {
                    continue; // is dirichlet already, nothing to do
                }

                bool jump_in_neighborbood = true;
                if (helper_pixels_jumps.at<uchar>(  i,   j) == 0 &&
                        helper_pixels_jumps.at<uchar>(i-1,   j) == 0 &&
                        helper_pixels_jumps.at<uchar>(i+1,   j) == 0 &&
                        helper_pixels_jumps.at<uchar>(  i, j-1) == 0 &&
                        helper_pixels_jumps.at<uchar>(  i, j+1) == 0 &&
                        helper_pixels_jumps.at<uchar>(i-1, j-1) == 0 &&
                        helper_pixels_jumps.at<uchar>(i+1, j+1) == 0 &&
                        helper_pixels_jumps.at<uchar>(i-1, j+1) == 0 &&
                        helper_pixels_jumps.at<uchar>(i+1, j-1) == 0)
                {
                    jump_in_neighborbood = false;
                }

                if (jump_in_neighborbood)
                {
                    unsigned int lx = j-1;
                    unsigned int ly = i;
                    unsigned int tx = j;
                    unsigned int ty = i-1;
                    unsigned int rx = j+1;
                    unsigned int ry = i;
                    unsigned int bx = j;
                    unsigned int by = i+1;
                    bool jump_l = false;
                    bool jump_t = false;
                    bool jump_r = false;
                    bool jump_b = false;
                    if (lx != helper_mapping_neighborhood[i][j][0][1]) jump_l = true;
                    lx = helper_mapping_neighborhood[i][j][0][1];
                    if (ly != helper_mapping_neighborhood[i][j][0][0]) jump_l = true;
                    ly = helper_mapping_neighborhood[i][j][0][0];
                    if (tx != helper_mapping_neighborhood[i][j][1][1]) jump_t = true;
                    tx = helper_mapping_neighborhood[i][j][1][1];
                    if (ty != helper_mapping_neighborhood[i][j][1][0]) jump_t = true;
                    ty = helper_mapping_neighborhood[i][j][1][0];
                    if (rx != helper_mapping_neighborhood[i][j][2][1]) jump_r = true;
                    rx = helper_mapping_neighborhood[i][j][2][1];
                    if (ry != helper_mapping_neighborhood[i][j][2][0]) jump_r = true;
                    ry = helper_mapping_neighborhood[i][j][2][0];
                    if (bx != helper_mapping_neighborhood[i][j][3][1]) jump_b = true;
                    bx = helper_mapping_neighborhood[i][j][3][1];
                    if (by != helper_mapping_neighborhood[i][j][3][0]) jump_b = true;
                    by = helper_mapping_neighborhood[i][j][3][0];

                    if ( jump_l && current_mask_CC.at<uchar>(ly, lx) == 0 )
                    {
                        if (j > 0)
                        {
                            helper_dirichlet_mask.at<uchar>(i, j-1) = 255;
                        }
                    }
                    if ( jump_t && current_mask_CC.at<uchar>(ty, tx) == 0 )
                    {
                        if (i > 0)
                        {
                            helper_dirichlet_mask.at<uchar>(i-1, j) = 255;
                        }
                    }
                    if ( jump_r && current_mask_CC.at<uchar>(ry, rx) == 0 )
                    {
                        if (j < cols-1)
                        {
                            helper_dirichlet_mask.at<uchar>(i, j+1) = 255;
                        }
                    }
                    if ( jump_b && current_mask_CC.at<uchar>(by, bx) == 0 )
                    {
                        if (i < rows-1)
                        {
                            helper_dirichlet_mask.at<uchar>(i+1, j) = 255;
                        }
                    }
                }

                bool on_boundary = false;
                if (current_mask_CC.at<uchar>(  i,   j) == 0 &&
                        (current_mask_CC.at<uchar>(i-1,   j) == 255 ||
                         current_mask_CC.at<uchar>(i+1,   j) == 255 ||
                         current_mask_CC.at<uchar>(  i, j-1) == 255 ||
                         current_mask_CC.at<uchar>(  i, j+1) == 255 ||
                         current_mask_CC.at<uchar>(i-1, j-1) == 255 ||
                         current_mask_CC.at<uchar>(i+1, j+1) == 255 ||
                         current_mask_CC.at<uchar>(i-1, j+1) == 255 ||
                         current_mask_CC.at<uchar>(i+1, j-1) == 255))
                {
                    on_boundary = true;

                }

                if ( on_boundary )
                {
                    if ( !jump_in_neighborbood )
                    {
                        helper_dirichlet_mask.at<uchar>(i, j) = 255;
                    }
                }
            }
        }

        // dilatation dirichlet mask
        cv::dilate(helper_dirichlet_mask, helper_dirichlet_mask, cv::Mat(), cv::Point(-1, -1), 2);
        // for debugging
        // cv::imwrite("DEBUG_DIRICHLET.png", helper_dirichlet_mask);

        // texture processing
        graphene::utility::Texture_processing texture_processing;
        bool resized_local = false;
        // if (!texture_processing.seamless_clone(texture_opencv, src, current_mask_CC, resized, false, helper_dirichlet_mask)) TODO SO WAR DAS VORHER, ABER GEHT NICHT
        if (!texture_processing.seamless_clone(texture_opencv, src, current_mask_CC, resized_local, false, helper_dirichlet_mask, helper_mapping_neighborhood))
        {
            std::cerr << "Texture_processing::repair_armpit_texture: [ERROR] seamless_clone failed." << std::endl;
            return false;
        }

    }

    const unsigned int new_w = texture_opencv.cols;
    const unsigned int new_h = texture_opencv.rows;
    if (new_w != mesh_color_texture->width_ || new_h != mesh_color_texture->height_)
    {
        mesh_color_texture->width_  = new_w;
        mesh_color_texture->height_ = new_h;

        mesh_color_texture->data_.resize(new_w * new_h * 4);
    }

    opencv_to_texture(texture_opencv, mesh_color_texture);

    mesh_node.update_textures();

    return true;
}

//-----------------------------------------------------------------------------

bool
Texture_processing::
adjust_eye_and_teeth_luminance(
        const std::string &fn_mask_face_and_ears,
        const std::string &fn_mask_eyes_and_teeth_inverse,
        bool invert_target_mask,
        bool invert_direction,
        scene_graph::Surface_mesh_node &mesh_node)
{

    gl::Texture* color_texture = mesh_node.get_texture(gl::TT_COLOR);
    if (color_texture == nullptr)
    {
        std::cerr << "Texture_processing::adjust_eye_and_teeth_luminance: [ERROR] Template has no texture." << std::endl;
        return false;
    }

    // target image
    cv::Mat texture_opencv = texture_to_opencv(color_texture);

    // target mask
    cv::Mat target_mask;
    std::string filename_target_mask = fn_mask_eyes_and_teeth_inverse;
    if (filename_target_mask.empty())
    {
        target_mask = cv::Mat::ones(texture_opencv.size(), 0)*255;
    }
    else
    {
        target_mask = cv::imread(filename_target_mask.c_str(), cv::IMREAD_GRAYSCALE);
        if( target_mask.empty() )
        {
            std::cerr << "Texture_processing::adjust_eye_and_teeth_luminance: [ERROR] Could not load target mask with filename: \""<< filename_target_mask << "\"." << std::endl;
            return false;
        }
    }
    if (invert_target_mask)
    {
        cv::subtract(cv::Scalar::all(255), target_mask, target_mask);
    }

    // texture processing
    graphene::utility::Texture_processing texture_processing;
    bool resized = false;

    // source mask
    cv::Mat source_mask;
    std::string filename_source_mask = fn_mask_face_and_ears;
    if (filename_source_mask.empty())
    {
        source_mask = cv::Mat::ones(texture_opencv.size(), 0)*255;
    }
    else
    {
        source_mask = cv::imread(filename_source_mask.c_str(), 0);
        if( source_mask.empty() )
        {
            std::cerr << "Texture_processing::adjust_eye_and_teeth_luminance: [ERROR] Could not load source mask with filename: \""<< filename_source_mask << "\"." << std::endl;
            return false;
        }
    }

    surface_mesh::Surface_mesh::Mesh_property<gl::Texture> color_texture_backup
            = mesh_node.mesh().get_mesh_property<gl::Texture>("m:color_texture_backup");


    if( !color_texture_backup )
    {
        std::cerr << "Texture_processing::adjust_eye_and_teeth_luminance: [ERROR] No backup texture found." << std::endl;
        return false;
    }

    cv::Mat texture_backup_ = texture_to_opencv(&color_texture_backup[0]);

    if (!texture_processing.luminance_mean_shift(texture_backup_, texture_opencv, source_mask, target_mask, source_mask, resized, invert_direction))
    {
        std::cerr << "Texture_processing::adjust_eye_and_teeth_luminance: [ERROR] Could not adjust luminance." << std::endl;
        return false;
    }

    if (resized)
    {
        const unsigned int new_w = texture_opencv.cols;
        const unsigned int new_h = texture_opencv.rows;
        color_texture->width_  = new_w;
        color_texture->height_ = new_h;

        color_texture->data_.resize(new_w * new_h * 4);
    }

    opencv_to_texture(texture_opencv, color_texture);

    mesh_node.update_textures();

    return true;
}

//-----------------------------------------------------------------------------

bool
Texture_processing::adjust_iris_color(scene_graph::Character_node& mesh_node, const std::string& fn_frontal_image, const std::string& fn_contour_eyes)
{
    gl::Texture* texture = mesh_node.get_texture(gl::TT_COLOR);

    if (!texture)
    {
        printf("[ERROR] adjust_iris_color: Cannot get texture of mesh_node\n");
        return false;
    }

    cv::Mat template_texture = texture_to_opencv(texture);

    //GET scan eyes
    //get positions
    cv::Mat face_scan = cv::imread(fn_frontal_image, cv::IMREAD_COLOR);
    std::ifstream contour_eyes_ifs;
    contour_eyes_ifs.open(fn_contour_eyes, std::iostream::in | std::iostream::out);
    std::vector<Vec2i> contour_eyes(12);
    Vec2i left_eye_center(0, 0);
    Vec2i right_eye_center(0, 0);

    //calc center of left eye
    for(int i = 0; i < 6; i++)
    {
        contour_eyes_ifs >> contour_eyes[i][0] >> contour_eyes[i][1];
        left_eye_center[0] += contour_eyes[i][0];
        left_eye_center[1] += contour_eyes[i][1];
    }

    //calc center of right eye
    for(int i = 6; i < 12; i++)
    {
        contour_eyes_ifs >> contour_eyes[i][0] >> contour_eyes[i][1];
        right_eye_center[0] += contour_eyes[i][0];
        right_eye_center[1] += contour_eyes[i][1];
    }

    left_eye_center /= 6;
    right_eye_center /= 6;

    int left_eye_radius = 0;
    int right_eye_radius = 0;

    //calc radius of left eye
    for(int i = 0; i < 6; ++i)
    {
        int dist = norm(contour_eyes[i] - left_eye_center);
        if(dist > left_eye_radius)
        {
            left_eye_radius = dist;
        }
    }

    //calc radius of right eye
    for(int i = 6; i < 12; ++i)
    {
        int dist = norm(contour_eyes[i] - right_eye_center);
        if(dist > right_eye_radius)
        {
            right_eye_radius = dist;
        }
    }

    cv::Mat left_scan_eye = cv::Mat::zeros(cv::Size(2 * left_eye_radius, 2 * left_eye_radius), CV_8UC3);
    //extract iris and filter skin [LEFT]
    extract_eye(face_scan, left_scan_eye, left_eye_radius, left_eye_center);
    extract_iris(left_scan_eye, left_eye_radius);
    filter_skin(left_scan_eye);

    cv::Mat right_scan_eye = cv::Mat::zeros(cv::Size(2 * right_eye_radius, 2 * right_eye_radius), CV_8UC3);
    //extract iris and filter skin [RIGHT]
    extract_eye(face_scan, right_scan_eye, right_eye_radius, right_eye_center);
    extract_iris(right_scan_eye, right_eye_radius);
    filter_skin(right_scan_eye);


    //GET template iris
    int radius = 100;

    p2c::Config* cfg = p2c::Config::instance();
    std::string template_eye_left = cfg->tmpl().s_eyemesh_left;
    std::string template_eye_right = cfg->tmpl().s_eyemesh_right;
    std::string template_eye_mesh_name;

    // prepare character -----------------------------------------------------
    character::Character& inout_character = mesh_node.character();

    cv::Mat scan_eye;
    //two iterations for two eyes
    for(int i = 0; i < 2; ++i)
    {
        if(i < 1)
        {
            template_eye_mesh_name = template_eye_left;
            scan_eye = left_scan_eye.clone();
        }
        else
        {
            template_eye_mesh_name = template_eye_right;
            scan_eye = right_scan_eye.clone();
        }
        //calc center position of template eye in texture
        int  eye_mesh_index = inout_character.get_skin_idx(template_eye_mesh_name);
        auto eye_mesh    = inout_character.skins()[eye_mesh_index];
        auto tex_coords  = eye_mesh->h_texcoords_;
        auto halfedge    = eye_mesh->halfedge(graphene::surface_mesh::Surface_mesh::Vertex(200));
        auto tex_coord   = tex_coords[eye_mesh->opposite_halfedge(halfedge)];
        // Scale and invert y
        Vec2i center = template_texture.rows * Vec2f(tex_coord[0], 1.0 - tex_coord[1]);

        cv::Mat template_eye = cv::Mat::zeros(cv::Size(2 * radius, 2 * radius), CV_8UC3);
        extract_eye_template(template_texture, template_eye, radius, center);
        std::cout << "template eye size " << template_eye.size() << std::endl;
        extract_iris_template(template_eye, radius);

        std::cout << "template eye size " << template_eye.size() << std::endl;
        //DO histogram Matching
        float bin_width = 1.0f;

        //Texture
        int template_size = template_eye.rows * template_eye.cols;
        cv::Mat output_iris;
        template_eye.copyTo(output_iris);

        //Scanned eye
        int scan_size = scan_eye.rows * scan_eye.cols;

        //Change color space from BGR to HSV
        cv::cvtColor(template_eye, template_eye, CV_BGR2HSV);
        cv::cvtColor(scan_eye, scan_eye, CV_BGR2HSV);
        cv::cvtColor(output_iris, output_iris, CV_BGR2HSV);

        for(int channel = 0; channel < 2; ++channel) //HSV 2 Channel, don't change the value! Change only hue and saturation
        {

            // Generate the histograms
            int template_histogram[256];
            template_size = imhist(template_eye, template_histogram, channel);
            //histDisplay(template_histogram, "template_histogram");

            int scan_histogram[256];
            scan_size = imhist(scan_eye, scan_histogram, channel);
            //histDisplay(scan_histogram, "scan_histogram");

            float template_pdf[256];
            float scan_pdf[256];

            //Calc probability density function
            for(int j = 0; j < 256; ++j)
            {
                template_pdf[j] = (float) template_histogram[j] / template_size;
                scan_pdf[j] = (float) scan_histogram[j] / scan_size;
            }

            float template_cdf[256];
            float scan_cdf[256];

            template_cdf[0] = 0.0;
            scan_cdf[0] = 0.0;

            //Calc cumulative density function
            for(int k = 1; k < 256; ++k)
            {
                template_cdf[k] = template_cdf[k - 1] + template_pdf[k];
                scan_cdf[k] = scan_cdf[k - 1] + scan_pdf[k];
            }


            int match_function[256];
            int last_match = 0;
            //Find mapping function M
            for(int k = 0; k < 256; k++)
            {
                int match_val = 0;
                for(int j = 0; j < 256; ++j)
                {
                   if(scan_cdf[j] > template_cdf[k])
                   {
                      match_val = j;
                      last_match = match_val;
                        break;
                   }
                 }
                match_function[k] = std::max(last_match, match_val);
            }


            //Apply M on every pixel
            for(int k = 0; k < template_eye.rows; ++k)
            {
                for(int j = 0; j < template_eye.cols; ++j)
                {
                    int value = (int) template_eye.at<cv::Vec3b>(k, j)[channel];
                    if(template_eye.at<cv::Vec3b>(k, j)[2] > 5)
                        output_iris.at<cv::Vec3b>(k, j)[channel] = match_function[value];
                }
            }
        }

        cv::cvtColor(template_eye, template_eye, CV_HSV2BGR);
        cv::cvtColor(scan_eye, scan_eye, CV_HSV2BGR);
        cv::cvtColor(output_iris, output_iris, CV_HSV2BGR);

//TESTAUSGABE
#if 0
        cv::namedWindow("INPUT", cv::WINDOW_AUTOSIZE);
        cv::imshow("INPUT", template_eye);
        cv::imwrite("template_iris.png", template_eye);
        cv::namedWindow("REF", cv::WINDOW_AUTOSIZE);
        cv::imshow("REF", scan_eye);
        cv::imwrite("reference_iris.png", scan_eye);

        cv::namedWindow("OUTPUT", cv::WINDOW_AUTOSIZE);
        cv::imshow("OUTPUT", output_iris);
        cv::waitKey();
        cv::imwrite("output_iris.png", output_iris);
#endif

        //CHANGE iris in model texture
        int output_rows = output_iris.rows;
        int output_cols = output_iris.cols;
        for(int x = 0; x < template_texture.rows; ++x)
        {
            for(int y = 0; y < template_texture.cols; ++y)
            {
                Vec2i pixel(x, y);
                if(norm(pixel - center) <= radius - 2 && norm(pixel - center) >= 0)
                {
                    Vec2i new_center(output_rows / 2, output_cols / 2);
                    Vec2i new_pixel = new_center + (pixel - center);
                    if(new_pixel[0] > 0 && new_pixel[0] <= output_rows && new_pixel[1] > 0 && new_pixel[1] <= output_cols)
                    {
                        auto new_color = output_iris.at<cv::Vec3b>(new_pixel[1], new_pixel[0]);

                        if (new_color != cv::Vec3b(0,0,0))
                            template_texture.at<cv::Vec3b>(y, x) = new_color;

                        // template_texture.at<cv::Vec3b>(y, x)[0] = output_iris.at<cv::Vec3b>(new_pixel[1], new_pixel[0])[0];
                        // template_texture.at<cv::Vec3b>(y, x)[1] = output_iris.at<cv::Vec3b>(new_pixel[1], new_pixel[0])[1];
                        // template_texture.at<cv::Vec3b>(y, x)[2] = output_iris.at<cv::Vec3b>(new_pixel[1], new_pixel[0])[2];
                    }
                }
            }
        }
    }

#if 0
    cv::namedWindow("INPUT", cv::WINDOW_NORMAL);
    cv::imshow("INPUT", template_texture);
    cv::waitKey();
#endif
    opencv_to_texture(template_texture, texture);
    mesh_node.update_textures();
    return true;
}

//HELPER----------------------HISTOGRAM MATCHING---------------------------------------------------

/*
 * Filter the skin out of the image.
 */
void Texture_processing::filter_skin(cv::Mat& image)
{
    for(int i = 0; i < image.rows; ++i)
    {
        for(int j = 0; j < image.cols; ++j)
        {
            int c_r = image.at<cv::Vec3b>(j, i)[2];
            int c_g = image.at<cv::Vec3b>(j, i)[1];
            int c_b = image.at<cv::Vec3b>(j, i)[0];

            bool   cond1 = (c_r > 95 && c_g > 40 && c_b > 20);
            double delta = std::max(c_r, std::max(c_g, c_b)) - std::min(c_r, std::min(c_g, c_b));
            bool   cond2 = delta > 15;
            bool   cond3 = std::abs(c_r - c_g) > 15 && c_r > c_g && c_r > c_b;
            if ( cond1 && cond2 && cond3) // skin
            {
                 image.at<cv::Vec3b>(j, i)[0] = 0;
                 image.at<cv::Vec3b>(j, i)[1] = 0;
                 image.at<cv::Vec3b>(j, i)[2] = 0;
            }
        }
    }
}

/*
 * Extract a circle with radius radius and center point center from an image.
 */
void Texture_processing::extract_eye(cv::Mat& image, cv::Mat& eye_image, int radius, Vec2i center)
{
    for(int x = 0; x < image.rows; ++x)
    {
        for(int y = 0; y < image.cols; ++y)
        {
            Vec2i pixel(x, y);
            if(norm(pixel - center) <= radius && norm(pixel - center) >= 0)
            {
                Vec2i new_center(radius, radius);
                Vec2i new_pixel = new_center + (pixel - center);
                if(new_pixel[0] > 0 && new_pixel[0] <= 2 * radius && new_pixel[1] > 0 && new_pixel[1] <= 2 * radius)
                {
                    eye_image.at<cv::Vec3b>(new_pixel[1], new_pixel[0])[0] = image.at<cv::Vec3b>(y, x)[0];
                    eye_image.at<cv::Vec3b>(new_pixel[1], new_pixel[0])[1] = image.at<cv::Vec3b>(y, x)[1];
                    eye_image.at<cv::Vec3b>(new_pixel[1], new_pixel[0])[2] = image.at<cv::Vec3b>(y, x)[2];
                }
            }
        }
    }
}

void Texture_processing::extract_eye_template(cv::Mat& image, cv::Mat& eye_image, int radius, Vec2i center)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::circle(mask, cv::Point(center[0], center[1]), radius, cv::Scalar(255), -1, CV_AA);
    cv::Rect roi(cv::Point(center[0] - radius, center[1] - radius), cv::Point(center[0] + radius, center[1] + radius));

    image(roi).copyTo(eye_image, mask(roi));
}

void Texture_processing::extract_iris_template(cv::Mat& eye_image, int iris_radius)
{

    //
    // TODO(swenninger): cleanup
    //


#if 1
    int min_radius = 60;
    int max_radius = 90;
    cv::Vec2i image_center(eye_image.rows / 2, eye_image.cols / 2);
    //find iris (ellipse)
    cv::Mat scan_gray;
    cv::cvtColor(eye_image, scan_gray, cv::COLOR_BGR2GRAY);


    // for(int i = 0; i < scan_gray.rows; ++i)
    //     for(int j = 0; j < scan_gray.cols; ++j)
    //     {
    //         scan_gray.at<uchar>(i, j) = scan_gray.at<uchar>(i, j) * 2.5;
    //     }
    // cv::medianBlur(scan_gray, scan_gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(scan_gray, circles, cv::HOUGH_GRADIENT, 1,
                 scan_gray.rows/2,  // change this value to detect circles with different distances to each other
                 100, 10, min_radius, max_radius // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );

    std::cout << "Found " << circles.size() << " circles" << std::endl;

    cv::Vec2i iris_center;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Vec2i center(c[0], c[1]);
        if(norm(center - image_center) > 50)
            continue;
        int radius = c[2];
        iris_radius = radius;
        iris_center = center;
        cv::circle( scan_gray, iris_center, iris_radius, cv::Scalar(255), 3, cv::LINE_AA);
    }

#if 1
    for(int i = 0; i < eye_image.rows; ++i)
    {
        for(int j = 0; j < eye_image.cols; ++j)
        {
            cv::Vec2i pixel(i, j);
            if(! (cv::norm(pixel - iris_center) <= iris_radius))
            {
                eye_image.at<cv::Vec3b>(j, i)[0] = 0;
                eye_image.at<cv::Vec3b>(j, i)[1] = 0;
                eye_image.at<cv::Vec3b>(j, i)[2] = 0;
            }
        }
    }
#endif
#endif
}



/*
 * Find an ellipse in the images. Set all values outside to black.
 */
void Texture_processing::extract_iris(cv::Mat& eye_image, int& iris_radius)
{
    int min_radius = 60;
    int max_radius = 90;
    cv::Vec2i image_center(eye_image.rows / 2, eye_image.cols / 2);
    //find iris (ellipse)
    cv::Mat scan_gray;
    cv::cvtColor(eye_image, scan_gray, cv::COLOR_BGR2GRAY);
    for(int i = 0; i < scan_gray.rows; ++i)
        for(int j = 0; j < scan_gray.cols; ++j)
        {
            scan_gray.at<uchar>(i, j) = scan_gray.at<uchar>(i, j) * 2.5;
        }
    cv::medianBlur(scan_gray, scan_gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(scan_gray, circles, cv::HOUGH_GRADIENT, 1,
                 scan_gray.rows/2,  // change this value to detect circles with different distances to each other
                 100, 10, min_radius, max_radius // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    cv::Vec2i iris_center;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Vec2i center(c[0], c[1]);
        if(norm(center - image_center) > 50)
            continue;
        int radius = c[2];
        iris_radius = radius;
        iris_center = center;
        //cv::circle( eye_image, iris_center, iris_radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }

    for(int i = 0; i < eye_image.rows; ++i)
    {
        for(int j = 0; j < eye_image.cols; ++j)
        {
            cv::Vec2i pixel(i, j);
            if(! (cv::norm(pixel - iris_center) <= iris_radius))
            {
                eye_image.at<cv::Vec3b>(j, i)[0] = 0;
                eye_image.at<cv::Vec3b>(j, i)[1] = 0;
                eye_image.at<cv::Vec3b>(j, i)[2] = 0;
            }
        }
    }
}

/*
 * Calculate a histogram of an image.
 */
int Texture_processing::imhist(cv::Mat image, int histogram[], int channel)
{

    // initialize all intensity values to 0
    for(int i = 0; i < 256; i++)
    {
        histogram[i] = 0;
    }

    int number_of_used_pixels = 0;
    // calculate the number of pixels for each intensity value
    for(int y = 0; y < image.rows; y++)
        for(int x = 0; x < image.cols; x++)
        {
            //
            if((int)image.at<cv::Vec3b>(y,x)[2] < 5) // don't use to small values -> don't change the pupil
            {
                continue;
            }
            histogram[(int)image.at<cv::Vec3b>(y,x)[channel]]++;
            number_of_used_pixels++;
        }
    return number_of_used_pixels;
}

/*
 *  Display a histogram. Used for testing.
*/
void Texture_processing::histDisplay(int histogram[], const char* name)
{
    int hist[256];
    for(int i = 0; i < 256; i++)
    {
        hist[i]=histogram[i];
    }
    // draw the histograms
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double) hist_w/256);

    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(255, 255, 255));

    // find the maximum intensity element from histogram
    int max = hist[0];
    for(int i = 1; i < 256; i++){
        if(max < hist[i]){
            max = hist[i];
        }
    }

    // normalize the histogram between 0 and histImage.rows

    for(int i = 0; i < 256; i++){
        hist[i] = ((double)hist[i]/max)*histImage.rows;
    }


    // draw the intensity line for histogram
    for(int i = 0; i < 256; i++)
    {
        cv::line(histImage, cv::Point(bin_w*(i), hist_h),
                              cv::Point(bin_w*(i), hist_h - hist[i]),
             cv::Scalar(0,0,0), 1, 8, 0);
    }

    // display histogram
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
    cv::imshow(name, histImage);
    cv::waitKey();
}

//-----------------------------------------------------------------------------


bool
Texture_processing::
gaussian_blur(cv::Mat& image, const cv::Mat& image_mask, const unsigned int size, bool& resized) const
{
    const cv::Size image_size_before = image.size();

    if ( (size % 2) != 1 )
    {
        std::cerr << "[ERROR] size must be odd!" << std::endl;
        return false;
    }

    if ( image_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] image mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_mask.data )
    {
        std::cerr << "[ERROR] loading image mask! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_rows   = image.rows;
    const unsigned int image_cols   = image.cols;
    const float        image_ratio  = image_cols / image_rows;
    const cv::Size     image_size(image_cols, image_rows);

    const unsigned int image_mask_rows  = image_mask.rows;
    const unsigned int image_mask_cols  = image_mask.cols;
    const float        image_mask_ratio = image_mask_cols / image_mask_rows;
    const cv::Size     image_mask_size(image_mask_cols, image_mask_rows);

    if (image_ratio != image_mask_ratio)
    {
        std::cerr << "[ERROR] image and mask have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image      = image.clone();
    cv::Mat local_image_mask = image_mask.clone();

    if (image_rows != image_mask_rows)
    {
        if (image_rows > image_mask_rows)
        {
            cv::resize(local_image_mask, local_image_mask, image_size);
        }
        else
        {
            cv::resize(local_image, local_image, image_mask_size);
        }
    }

    cv::Mat result = local_image.clone();

    cv::GaussianBlur(local_image, result, cv::Size(size, size), 0, 0);

    const unsigned int rows = local_image.rows;
    const unsigned int cols = local_image.cols;
    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            if ( local_image_mask.at<uchar>(i, j) == 0 )
            {
                result.at<cv::Vec3b>(i, j) = local_image.at<cv::Vec3b>(i, j);
            }
        }
    }

    const cv::Size image_size_after = image.size();

    if (image_size_before != image_size_after)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    image = result.clone();

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
contrast_brightness(cv::Mat& image, const cv::Mat& image_mask, const double contrast_parameter, const double brightness_parameter, bool& resized) const
{
    const cv::Size image_size_before = image.size();

    if ( image_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] image mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_mask.data )
    {
        std::cerr << "[ERROR] loading image mask! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_rows   = image.rows;
    const unsigned int image_cols   = image.cols;
    const float        image_ratio  = image_cols / image_rows;
    const cv::Size     image_size(image_cols, image_rows);

    const unsigned int image_mask_rows  = image_mask.rows;
    const unsigned int image_mask_cols  = image_mask.cols;
    const float        image_mask_ratio = image_mask_cols / image_mask_rows;
    const cv::Size     image_mask_size(image_mask_cols, image_mask_rows);

    if (image_ratio != image_mask_ratio)
    {
        std::cerr << "[ERROR] image and mask have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image      = image.clone();
    cv::Mat local_image_mask = image_mask.clone();

    if (image_rows != image_mask_rows)
    {
        if (image_rows > image_mask_rows)
        {
            cv::resize(local_image_mask, local_image_mask, image_size);
        }
        else
        {
            cv::resize(local_image, local_image, image_mask_size);
        }
    }

    cv::Mat result = cv::Mat::zeros( local_image.size(), local_image.type() );

    // do the operation new_image(i,j) = alpha*image(i,j) + beta
    for( int y = 0; y < local_image.rows; y++ )
    {
        for( int x = 0; x < local_image.cols; x++ )
        {
            const double weight = local_image_mask.at<uchar>(y, x) / 255.0; // in [0 , 1]

            for( int c = 0; c < 3; c++ )
            {
                const double mod_contrast_parameter = 1.0 + weight*(contrast_parameter - 1.0);

                result.at<cv::Vec3b>(y,x)[c] =
                    cv::saturate_cast<uchar>( mod_contrast_parameter*( local_image.at<cv::Vec3b>(y,x)[c] ) + weight*brightness_parameter );
            }
        }
    }

    const cv::Size image_size_after = image.size();

    if (image_size_before != image_size_after)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    image = result.clone();

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
kmeans(cv::Mat& image, const int cluster_count) const
{
    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( cluster_count < 1 )
    {
        std::cerr << "[ERROR] #clusters must be positiv! Aborting ..." << std::endl;
        return false;
    }

    cv::Mat samples(image.rows * image.cols, 3, CV_32F);
    for( int y = 0; y < image.rows; y++ )
    {
        for( int x = 0; x < image.cols; x++ )
        {
            for( int z = 0; z < 3; z++)
            {
                samples.at<float>(y + x*image.rows, z) = image.at<cv::Vec3b>(y,x)[z];
            }
        }
    }

    cv::Mat labels;
    int attempts = 5;
    cv::Mat centers;
    cv::kmeans(samples, cluster_count, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers );

    cv::Mat result( image.size(), image.type() );
    for( int y = 0; y < image.rows; y++ )
    {
        for( int x = 0; x < image.cols; x++ )
        {
            int cluster_idx = labels.at<int>(y + x*image.rows,0);
            result.at<cv::Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
            result.at<cv::Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
            result.at<cv::Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        }
    }

    image = result.clone();

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
linear_blend(cv::Mat& image, const cv::Mat& image_A, const cv::Mat& image_B, const cv::Mat& image_mask, const double val, bool& resized) const
{
    if ( val < 0.0 || val > 1.0 )
    {
        std::cerr << "[ERROR] alpha must be between 0 and 1! Aborting ..." << std::endl;
        return false;
    }

    if ( image_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] image mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_A.data )
    {
        std::cerr << "[ERROR] loading image A! Aborting ..." << std::endl;
        return false;
    }
    if( !image_B.data )
    {
        std::cerr << "[ERROR] loading image B! Aborting ..." << std::endl;
        return false;
    }
    if( !image_mask.data )
    {
        std::cerr << "[ERROR] loading image mask! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_rows   = image.rows;
    const unsigned int image_cols   = image.cols;
    const float        image_ratio  = image_cols / image_rows;
    const cv::Size     image_size(image_cols, image_rows);

    const unsigned int imageA_rows       = image_A.rows;
    const unsigned int imageA_cols       = image_A.cols;
    const float        imageA_ratio      = imageA_cols / imageA_rows;

    const unsigned int imageB_rows       = image_B.rows;
    const unsigned int imageB_cols       = image_B.cols;
    const float        imageB_ratio      = imageB_cols / imageB_rows;

    const unsigned int image_mask_rows  = image_mask.rows;
    const unsigned int image_mask_cols  = image_mask.cols;
    const float        image_mask_ratio = image_mask_cols / image_mask_rows;

    if (imageA_ratio     != image_ratio ||
        imageB_ratio     != image_ratio ||
        image_mask_ratio != image_ratio)
    {
        std::cerr << "[ERROR] images have incompatible size." << std::endl;
        return false;
    }

    cv::Mat local_image      = image.clone();
    cv::Mat local_image_A    = image_A.clone();
    cv::Mat local_image_B    = image_B.clone();
    cv::Mat local_image_mask = image_mask.clone();

    cv::Size largest_size = image_size;
    if (local_image_A.size().width > largest_size.width)
    {
        largest_size = local_image_A.size();
    }
    if (local_image_B.size().width > largest_size.width)
    {
        largest_size = local_image_B.size();
    }
    if (local_image_mask.size().width > largest_size.width)
    {
        largest_size = local_image_mask.size();
    }

    cv::resize(local_image, local_image, largest_size);
    cv::resize(local_image_A, local_image_A, largest_size);
    cv::resize(local_image_B, local_image_B, largest_size);
    cv::resize(local_image_mask, local_image_mask, largest_size);

    const unsigned int rows = local_image.rows;
    const unsigned int cols = local_image.cols;

    cv::Mat prev_img = local_image.clone();

    const double beta = ( 1.0 - val );
    cv::addWeighted( local_image_A, beta, local_image_B, val, 0.0, local_image);

    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            const double weight = local_image_mask.at<uchar>(i, j) / 255.0; // in [0 , 1]

            local_image.at<cv::Vec3b>(i, j) = weight*local_image.at<cv::Vec3b>(i, j) + (1.0 - weight)*prev_img.at<cv::Vec3b>(i, j);
        }
    }

    if (largest_size != image_size)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    image = local_image.clone();

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
copy_and_paste(cv::Mat& image, const cv::Mat& image_src, bool& resized) const
{
    const cv::Mat image_mask(image.size(), CV_8UC1, cv::Scalar(255));

    return copy_and_paste(image, image_src, image_mask, resized);
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
copy_and_paste(cv::Mat& image, const cv::Mat& image_src, const cv::Mat& image_mask, bool& resized) const
{
    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_src.data )
    {
        std::cerr << "[ERROR] loading source image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_mask.data )
    {
        std::cerr << "[ERROR] loading image mask! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_rows   = image.rows;
    const unsigned int image_cols   = image.cols;
    const float        image_ratio  = image_cols / image_rows;
    const cv::Size     image_size(image_cols, image_rows);

    const unsigned int image_src_rows  = image_src.rows;
    const unsigned int image_src_cols  = image_src.cols;
    const float        image_src_ratio = image_src_cols / image_src_rows;

    const unsigned int image_mask_rows  = image_mask.rows;
    const unsigned int image_mask_cols  = image_mask.cols;
    const float        image_mask_ratio = image_mask_cols / image_mask_rows;

    if (image_src_ratio  != image_ratio ||
        image_mask_ratio != image_ratio)
    {
        std::cerr << "[ERROR] images have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image      = image.clone();
    cv::Mat local_image_src  = image_src.clone();
    cv::Mat local_image_mask = image_mask.clone();

    cv::Size largest_size = image_size;
    if (local_image_src.size().width > largest_size.width)
    {
        largest_size = local_image_src.size();
    }
    if (local_image_mask.size().width > largest_size.width)
    {
        largest_size = local_image_mask.size();
    }

    cv::resize(local_image, local_image, largest_size);
    cv::resize(local_image_src, local_image_src, largest_size);
    cv::resize(local_image_mask, local_image_mask, largest_size);

    cv::Mat result = local_image.clone();

    const unsigned int rows = local_image_src.rows;
    const unsigned int cols = local_image_src.cols;

    const int channels_mask = local_image_mask.channels();

    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            if (channels_mask == 3)
            {
                const double tx = (double) local_image_mask.at<cv::Vec3b>(i, j)[0] / 255.0;
                const double ty = (double) local_image_mask.at<cv::Vec3b>(i, j)[1] / 255.0;
                const double tz = (double) local_image_mask.at<cv::Vec3b>(i, j)[2] / 255.0;

                result.at<cv::Vec3b>(i, j)[0] = tx * local_image_src.at<cv::Vec3b>(i, j)[0] + (1.0 - tx) * local_image.at<cv::Vec3b>(i, j)[0];
                result.at<cv::Vec3b>(i, j)[1] = ty * local_image_src.at<cv::Vec3b>(i, j)[1] + (1.0 - ty) * local_image.at<cv::Vec3b>(i, j)[1];
                result.at<cv::Vec3b>(i, j)[2] = tz * local_image_src.at<cv::Vec3b>(i, j)[2] + (1.0 - tz) * local_image.at<cv::Vec3b>(i, j)[2];
            }
            else if (channels_mask == 1)
            {
                const double t = (double) local_image_mask.at<uchar>(i, j) / 255.0;

                result.at<cv::Vec3b>(i, j) = t * local_image_src.at<cv::Vec3b>(i, j) + (1.0 - t) * local_image.at<cv::Vec3b>(i, j);
            }
            else
            {
                std::cerr << "[ERROR] Incompatible number of channels." << std::endl;
                return false;
            }
        }
    }

    if (largest_size != image_size)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    image = result.clone();

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
seamless_clone(cv::Mat& image, const cv::Mat& image_src, const cv::Mat& image_mask, bool& resized, const bool use_opencv_PIE, const cv::Mat& dirichlet_mask,
               const std::vector< std::vector< std::vector< std::array<int, 2> > > >& mapping_neighborhood) const
{
    if ( image_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] image mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_src.data )
    {
        std::cerr << "[ERROR] loading source image! Aborting ..." << std::endl;
        return false;
    }

    if( !image_mask.data )
    {
        std::cerr << "[ERROR] loading image mask! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_rows   = image.rows;
    const unsigned int image_cols   = image.cols;
    const float        image_ratio  = image_cols / image_rows;
    const cv::Size     image_size(image_cols, image_rows);

    const unsigned int image_src_rows  = image_src.rows;
    const unsigned int image_src_cols  = image_src.cols;
    const float        image_src_ratio = image_src_cols / image_src_rows;

    const unsigned int image_mask_rows  = image_mask.rows;
    const unsigned int image_mask_cols  = image_mask.cols;
    const float        image_mask_ratio = image_mask_cols / image_mask_rows;

    unsigned int dirichlet_mask_rows;
    unsigned int dirichlet_mask_cols;
    float        dirichlet_mask_ratio;
    if (dirichlet_mask.data)
    {
        dirichlet_mask_rows  = dirichlet_mask.rows;
        dirichlet_mask_cols  = dirichlet_mask.cols;
        dirichlet_mask_ratio = dirichlet_mask_cols / dirichlet_mask_rows;
    }

    if (image_src_ratio  != image_ratio ||
        image_mask_ratio != image_ratio)
    {
        std::cerr << "[ERROR] images have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image          = image.clone();
    cv::Mat local_image_src      = image_src.clone();
    cv::Mat local_image_mask     = image_mask.clone();
    cv::Mat local_dirichlet_mask;
    if (dirichlet_mask.data)
    {
        local_dirichlet_mask = dirichlet_mask.clone();
    }

    cv::Size largest_size = image_size;
    if (local_image_src.size().width > largest_size.width)
    {
        largest_size = local_image_src.size();
    }
    if (local_image_mask.size().width > largest_size.width)
    {
        largest_size = local_image_mask.size();
    }
    if (dirichlet_mask.data)
    {
        if (local_dirichlet_mask.size().width > largest_size.width)
        {
            largest_size = local_dirichlet_mask.size();
        }
    }

    cv::resize(local_image, local_image, largest_size);
    cv::resize(local_image_src, local_image_src, largest_size);
    cv::resize(local_image_mask, local_image_mask, largest_size);
    if (dirichlet_mask.data)
    {
        cv::resize(local_dirichlet_mask, local_dirichlet_mask, largest_size);
    }

    // the location of the center of the src in the dst (center of white mask region)
    std::vector<cv::Point> points;
    for (unsigned int i = 0; i < local_image_mask.rows; ++i)
    {
        for (unsigned int j = 0; j < local_image_mask.cols; ++j)
        {
            if ( local_image_mask.at<uchar>(i, j) == 255 )
            {
                points.push_back(cv::Point(j, i));
            }
        }
    }
    cv::Rect  bbox = cv::boundingRect(points);
    cv::Point center(bbox.x + bbox.width/2, bbox.y + bbox.height/2);


    // seamlessly clone src into dst and put the results in output
    cv::Mat seamless_clone;
    if (use_opencv_PIE)
    {
        cv::seamlessClone(local_image_src, local_image, local_image_mask, center, seamless_clone, cv::NORMAL_CLONE);
    }
    else
    {
        Poisson_image_editing poisson_image_editing(local_image_src, local_image_mask, local_image, center.x, center.y, seamless_clone, local_dirichlet_mask, mapping_neighborhood);
    }

    if (largest_size != image_size)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    image = seamless_clone.clone();

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
luminance_mean_shift(const cv::Mat& source_image, cv::Mat& target_image,
                     const cv::Mat& source_mask, const cv::Mat& target_mask,
                     const cv::Mat& reference_mask, bool& resized,
                     const bool invert_direction) const
{
    if( !source_image.data )
    {
        std::cerr << "[ERROR] loading source image! Aborting ..." << std::endl;
        return false;
    }

    if( !target_image.data )
    {
        std::cerr << "[ERROR] loading target image! Aborting ..." << std::endl;
        return false;
    }

    if ( source_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] source mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if ( target_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] target mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if ( reference_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] reference mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_src_rows  = source_image.rows;
    const unsigned int image_src_cols  = source_image.cols;
    const float        image_src_ratio = image_src_cols / image_src_rows;

    const unsigned int image_tar_rows  = target_image.rows;
    const unsigned int image_tar_cols  = target_image.cols;
    const float        image_tar_ratio = image_tar_cols / image_tar_rows;
    const cv::Size     image_tar_size(image_tar_cols, image_tar_rows);

    const unsigned int image_src_mask_rows  = source_mask.rows;
    const unsigned int image_src_mask_cols  = source_mask.cols;
    const float        image_src_mask_ratio = image_src_mask_cols / image_src_mask_rows;

    const unsigned int image_tar_mask_rows  = target_mask.rows;
    const unsigned int image_tar_mask_cols  = target_mask.cols;
    const float        image_tar_mask_ratio = image_tar_mask_cols / image_tar_mask_rows;

    const unsigned int image_ref_mask_rows  = reference_mask.rows;
    const unsigned int image_ref_mask_cols  = reference_mask.cols;
    const float        image_ref_mask_ratio = image_ref_mask_cols / image_ref_mask_rows;

    if (image_src_ratio != image_tar_ratio ||
        image_src_ratio != image_src_mask_ratio ||
        image_src_ratio != image_tar_mask_ratio ||
        image_src_ratio != image_ref_mask_ratio)
    {
        std::cerr << "[ERROR] images have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image_src  = source_image.clone();
    cv::Mat local_image_tar  = target_image.clone();
    cv::Mat local_mask_src   = source_mask.clone();
    cv::Mat local_mask_tar   = target_mask.clone();
    cv::Mat local_mask_ref   = reference_mask.clone();

    cv::Size largest_size = image_tar_size;
    if (local_image_src.size().width > largest_size.width)
    {
        largest_size = local_image_src.size();
    }
    if (local_mask_src.size().width > largest_size.width)
    {
        largest_size = local_mask_src.size();
    }
    if (local_mask_tar.size().width > largest_size.width)
    {
        largest_size = local_mask_tar.size();
    }
    if (local_mask_ref.size().width > largest_size.width)
    {
        largest_size = local_mask_ref.size();
    }

    cv::resize(local_image_src, local_image_src, largest_size);
    cv::resize(local_image_tar, local_image_tar, largest_size);
    cv::resize(local_mask_src, local_mask_src, largest_size);
    cv::resize(local_mask_tar, local_mask_tar, largest_size);
    cv::resize(local_mask_ref, local_mask_ref, largest_size);

    cv::cvtColor(local_image_src, local_image_src, CV_BGR2Lab);
    cv::cvtColor(local_image_tar, local_image_tar, CV_BGR2Lab);

    std::vector<cv::Mat> src_mv;
    cv::split(local_image_src, src_mv);
    cv::Mat l_src = src_mv[0];
    cv::Mat a_src = src_mv[1];
    cv::Mat b_src = src_mv[2];

    std::vector<cv::Mat> tar_mv;
    cv::split(local_image_tar, tar_mv);
    cv::Mat l_tar = tar_mv[0];
    cv::Mat a_tar = tar_mv[1];
    cv::Mat b_tar = tar_mv[2];

    // shift luminance
    Channel_mean_shift channel_mean_shift(l_src,
                                          l_tar,
                                          local_mask_src,
                                          local_mask_tar,
                                          local_mask_ref,
                                          invert_direction);

    cv::Mat result;
    tar_mv.clear();
    tar_mv.push_back(l_tar);
    tar_mv.push_back(a_tar);
    tar_mv.push_back(b_tar);
    cv::merge(tar_mv, result);

    cv::cvtColor(result, result, CV_Lab2BGR);

    target_image = result.clone();

    if (largest_size != image_tar_size)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
luminance_mean_shift(const unsigned int mean_luminance, cv::Mat& target_image,
                     const cv::Mat& target_mask, const cv::Mat& reference_mask, bool& resized) const
{
    if( !target_image.data )
    {
        std::cerr << "[ERROR] loading target image! Aborting ..." << std::endl;
        return false;
    }

    if ( target_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] target mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if ( reference_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] reference mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_tar_rows  = target_image.rows;
    const unsigned int image_tar_cols  = target_image.cols;
    const float        image_tar_ratio = image_tar_cols / image_tar_rows;
    const cv::Size     image_tar_size(image_tar_cols, image_tar_rows);

    const unsigned int image_tar_mask_rows  = target_mask.rows;
    const unsigned int image_tar_mask_cols  = target_mask.cols;
    const float        image_tar_mask_ratio = image_tar_mask_cols / image_tar_mask_rows;

    const unsigned int image_ref_mask_rows  = reference_mask.rows;
    const unsigned int image_ref_mask_cols  = reference_mask.cols;
    const float        image_ref_mask_ratio = image_ref_mask_cols / image_ref_mask_rows;

    if (image_tar_ratio != image_tar_mask_ratio ||
        image_tar_ratio != image_ref_mask_ratio)
    {
        std::cerr << "[ERROR] images have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image_tar  = target_image.clone();
    cv::Mat local_mask_tar   = target_mask.clone();
    cv::Mat local_mask_ref   = reference_mask.clone();

    cv::Size largest_size = image_tar_size;
    if (local_mask_tar.size().width > largest_size.width)
    {
        largest_size = local_mask_tar.size();
    }
    if (local_mask_ref.size().width > largest_size.width)
    {
        largest_size = local_mask_ref.size();
    }

    cv::resize(local_image_tar, local_image_tar, largest_size);
    cv::resize(local_mask_tar, local_mask_tar, largest_size);
    cv::resize(local_mask_ref, local_mask_ref, largest_size);

    cv::cvtColor(local_image_tar, local_image_tar, CV_BGR2Lab);

    std::vector<cv::Mat> tar_mv;
    cv::split(local_image_tar, tar_mv);
    cv::Mat l_tar = tar_mv[0];
    cv::Mat a_tar = tar_mv[1];
    cv::Mat b_tar = tar_mv[2];

    // shift luminance
    Channel_mean_shift channel_mean_shift(mean_luminance,
                                          l_tar,
                                          local_mask_tar,
                                          local_mask_ref);

    cv::Mat result;
    tar_mv.clear();
    tar_mv.push_back(l_tar);
    tar_mv.push_back(a_tar);
    tar_mv.push_back(b_tar);
    cv::merge(tar_mv, result);

    cv::cvtColor(result, result, CV_Lab2BGR);

    target_image = result.clone();

    if (largest_size != image_tar_size)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
color_warp_rbf(const std::vector<Color_warp_rbf::Color_constraint>& color_constraints_lab, cv::Mat& target_image, const cv::Mat& target_mask, bool& resized) const
{
    if( color_constraints_lab.size() == 0 )
    {
        std::cerr << "[ERROR] no color constraints available! Aborting ..." << std::endl;
        return false;
    }

    if( !target_image.data )
    {
        std::cerr << "[ERROR] loading target image! Aborting ..." << std::endl;
        return false;
    }

    if ( target_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] target mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    // check ratios
    const unsigned int image_tar_rows  = target_image.rows;
    const unsigned int image_tar_cols  = target_image.cols;
    const float        image_tar_ratio = image_tar_cols / image_tar_rows;
    const cv::Size     image_tar_size(image_tar_cols, image_tar_rows);

    const unsigned int image_tar_mask_rows  = target_mask.rows;
    const unsigned int image_tar_mask_cols  = target_mask.cols;
    const float        image_tar_mask_ratio = image_tar_mask_cols / image_tar_mask_rows;

    if (image_tar_ratio != image_tar_mask_ratio)
    {
        std::cerr << "[ERROR] images have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image_tar  = target_image.clone();
    cv::Mat local_mask_tar   = target_mask.clone();

    cv::Size largest_size = image_tar_size;
    if (local_mask_tar.size().width > largest_size.width)
    {
        largest_size = local_mask_tar.size();
    }

    cv::resize(local_image_tar, local_image_tar, largest_size);
    cv::resize(local_mask_tar, local_mask_tar, largest_size);

    // color warping
    target_image = local_image_tar.clone();
    Color_warp_rbf color_warp_rbf(color_constraints_lab);
    color_warp_rbf.warp_colors(local_mask_tar, target_image);

    if (largest_size != image_tar_size)
    {
        resized = true;
    }
    else
    {
        resized = false;
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
generate_mask(const int dim_texture, graphene::surface_mesh::Surface_mesh& mesh, cv::Mat& image_mask) const
{
    // gray-scale mask
    image_mask = cv::Mat::zeros(dim_texture, dim_texture, CV_8UC1);
    std::cerr << "[DEBUG] image_mask.rows: " << image_mask.rows << std::endl;
    std::cerr << "[DEBUG] image_mask.cols: " << image_mask.cols << std::endl;

    auto vselected = mesh.get_vertex_property<bool>("v:selected");
    auto texcoords = mesh.get_halfedge_property<Texture_coordinate>("h:texcoord");

    if (!texcoords)
    {
        std::cerr << "[ERROR] Can't generate mask from selection. No texture corrdinates available." << std::endl;
        return false;
    }

    if (vselected)
    {
        for (auto f : mesh.faces())
        {
            auto fhit = mesh.halfedges(f);
            const auto h0 = *fhit;
            const auto h1 = *(++fhit);
            const auto h2 = *(++fhit);

            const auto v0 = mesh.to_vertex(h0);
            const auto v1 = mesh.to_vertex(h1);
            const auto v2 = mesh.to_vertex(h2);

            if (vselected[v0] && vselected[v1] && vselected[v2])
            {
                // paint white triangle
                const Vec2f corner_1 = Vec2f(texcoords[h0][0], 1.0 - texcoords[h0][1]) * dim_texture;
                const Vec2f corner_2 = Vec2f(texcoords[h1][0], 1.0 - texcoords[h1][1]) * dim_texture;
                const Vec2f corner_3 = Vec2f(texcoords[h2][0], 1.0 - texcoords[h2][1]) * dim_texture;

                cv::Point corner_points[1][3];
                corner_points[0][0] = cv::Point((int)corner_1[0], (int)corner_1[1]);
                corner_points[0][1] = cv::Point((int)corner_2[0], (int)corner_2[1]);
                corner_points[0][2] = cv::Point((int)corner_3[0], (int)corner_3[1]);

                const cv::Point* ppt[1] = { corner_points[0] };
                int npt[] = { 3 };

                cv::fillPoly(image_mask, ppt, npt, 1, cv::Scalar(255, 255, 255), 8);
            }
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
generate_selection(graphene::surface_mesh::Surface_mesh& mesh, const cv::Mat& image_mask) const
{
    if ( image_mask.channels() != 1 )
    {
        std::cerr << "[ERROR] image mask should have one channel only! Aborting ..." << std::endl;
        return false;
    }

    if( !image_mask.data )
    {
        std::cerr << "[ERROR] loading image mask! Aborting ..." << std::endl;
        return false;
    }

    const unsigned int rows = image_mask.rows;
    const unsigned int cols = image_mask.cols;

    auto vselected = mesh.vertex_property<bool>("v:selected");
    auto texcoords = mesh.get_halfedge_property<Texture_coordinate>("h:texcoord");

    if (!texcoords)
    {
        std::cerr << "[ERROR] Can't generate selection from mask. No texture corrdinates available." << std::endl;
        return false;
    }

    for (auto v : mesh.vertices())
    {
        vselected[v] = false;
    }

    for (auto f : mesh.faces())
    {
        auto fhit = mesh.halfedges(f);
        const auto h0 = *fhit;
        const auto h1 = *(++fhit);
        const auto h2 = *(++fhit);

        const auto v0 = mesh.to_vertex(h0);
        const auto v1 = mesh.to_vertex(h1);
        const auto v2 = mesh.to_vertex(h2);

        // first texcoord
        Texture_coordinate tex0 = texcoords[h0];
        tex0[1] = 1.0 - tex0[1];
        unsigned int pix0_x = tex0[0] * (cols-1);
        unsigned int pix0_y = tex0[1] * (rows-1);
        if ( image_mask.at<uchar>(pix0_y, pix0_x) != 0 )
        {
            vselected[v0] = true;
        }

        // second texcoord
        Texture_coordinate tex1 = texcoords[h1];
        tex1[1] = 1.0 - tex1[1];
        unsigned int pix1_x = tex1[0] * (cols-1);
        unsigned int pix1_y = tex1[1] * (rows-1);
        if ( image_mask.at<uchar>(pix1_y, pix1_x) != 0 )
        {
            vselected[v1] = true;
        }

        // third texcoord
        Texture_coordinate tex2 = texcoords[h2];
        tex2[1] = 1.0 - tex2[1];
        unsigned int pix2_x = tex2[0] * (cols-1);
        unsigned int pix2_y = tex2[1] * (rows-1);
        if ( image_mask.at<uchar>(pix2_y, pix2_x) != 0 )
        {
            vselected[v2] = true;
        }
    }

    return true;
}


//-----------------------------------------------------------------------------


bool
Texture_processing::
compute_mask_from_black_areas(cv::Mat& out_mask, const cv::Mat& image, const cv::Mat& source_mask) const
{
    if( !image.data )
    {
        std::cerr << "[ERROR] loading image! Aborting ..." << std::endl;
        return false;
    }

    if( !source_mask.data )
    {
        std::cerr << "[ERROR] loading source mask! Aborting ..." << std::endl;
        return false;
    }

    out_mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

    // check ratios
    const unsigned int image_rows   = image.rows;
    const unsigned int image_cols   = image.cols;
    const float        image_ratio  = image_cols / image_rows;
    const cv::Size     image_size(image_cols, image_rows);

    const unsigned int out_mask_rows  = out_mask.rows;
    const unsigned int out_mask_cols  = out_mask.cols;
    const float        out_mask_ratio = out_mask_cols / out_mask_rows;
    const cv::Size     out_mask_size(out_mask_cols, out_mask_rows);

    const unsigned int source_mask_rows  = source_mask.rows;
    const unsigned int source_mask_cols  = source_mask.cols;
    const float        source_mask_ratio = source_mask_cols / source_mask_rows;

    if (out_mask_ratio    != image_ratio ||
        source_mask_ratio != image_ratio)
    {
        std::cerr << "[ERROR] images have incompatible ratios." << std::endl;
        return false;
    }

    cv::Mat local_image       = image.clone();
    cv::Mat local_out_mask    = out_mask.clone();
    cv::Mat local_source_mask = source_mask.clone();

    cv::Size largest_size = image_size;
    if (local_out_mask.size().width > largest_size.width)
    {
        largest_size = local_out_mask.size();
    }
    if (local_source_mask.size().width > largest_size.width)
    {
        largest_size = local_source_mask.size();
    }

    cv::resize(local_image, local_image, largest_size);
    cv::resize(local_out_mask, local_out_mask, largest_size);
    cv::resize(local_source_mask, local_source_mask, largest_size);

    cv::Mat result = local_out_mask.clone();

    const unsigned int rows = local_image.rows;
    const unsigned int cols = local_image.cols;

    const int channels_mask = local_source_mask.channels();

    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            if (channels_mask == 3)
            {
                bool candidate = false;
                if ( local_source_mask.at<cv::Vec3b>(i, j)[0] == 255 &&
                     local_source_mask.at<cv::Vec3b>(i, j)[1] == 255 &&
                     local_source_mask.at<cv::Vec3b>(i, j)[2] == 255 )
                {
                    candidate = true;
                }

                if ( candidate && local_image.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0) )
                {
                    result.at<uchar>(i, j) = 255;
                }
            }
            else if (channels_mask == 1)
            {
                bool candidate = false;
                if ( local_source_mask.at<uchar>(i, j) == 255 )
                {
                    candidate = true;
                }

                if ( candidate && local_image.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0) )
                {
                    result.at<uchar>(i, j) = 255;
                }
            }
            else
            {
                std::cerr << "[ERROR] Incompatible number of channels." << std::endl;
                return false;
            }
        }
    }

    out_mask = result.clone();

    return true;
}


bool Texture_processing::round_to_next_valid_pixel(const double pix_i, const double pix_j, const cv::Mat &texture_colored, Vec2ui &result) const
{
    const int pix_i_ver1 = pix_i;
    const int pix_j_ver1 = pix_j;

    const int pix_i_ver2 = pix_i + 1;
    const int pix_j_ver2 = pix_j;

    const int pix_i_ver3 = pix_i;
    const int pix_j_ver3 = pix_j + 1;

    const int pix_i_ver4 = pix_i + 1;
    const int pix_j_ver4 = pix_j + 1;

    const int pix_i_ver5 = pix_i - 1;
    const int pix_j_ver5 = pix_j;

    const int pix_i_ver6 = pix_i;
    const int pix_j_ver6 = pix_j - 1;

    const int pix_i_ver7 = pix_i - 1;
    const int pix_j_ver7 = pix_j - 1;

    const int pix_i_ver8 = pix_i + 1;
    const int pix_j_ver8 = pix_j - 1;

    const int pix_i_ver9 = pix_i - 1;
    const int pix_j_ver9 = pix_j + 1;

    const cv::Vec3b pix_ver1 = texture_colored.at<cv::Vec3b>(pix_i_ver1, pix_j_ver1);
    int faceID_ver1_pixel = pix_ver1[2] * 256 * 256 + pix_ver1[1] * 256 + pix_ver1[0];
    faceID_ver1_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver2 = texture_colored.at<cv::Vec3b>(pix_i_ver2, pix_j_ver2);
    int faceID_ver2_pixel = pix_ver2[2] * 256 * 256 + pix_ver2[1] * 256 + pix_ver2[0];
    faceID_ver2_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver3 = texture_colored.at<cv::Vec3b>(pix_i_ver3, pix_j_ver3);
    int faceID_ver3_pixel = pix_ver3[2] * 256 * 256 + pix_ver3[1] * 256 + pix_ver3[0];
    faceID_ver3_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver4 = texture_colored.at<cv::Vec3b>(pix_i_ver4, pix_j_ver4);
    int faceID_ver4_pixel = pix_ver4[2] * 256 * 256 + pix_ver4[1] * 256 + pix_ver4[0];
    faceID_ver4_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver5 = texture_colored.at<cv::Vec3b>(pix_i_ver5, pix_j_ver5);
    int faceID_ver5_pixel = pix_ver5[2] * 256 * 256 + pix_ver5[1] * 256 + pix_ver5[0];
    faceID_ver5_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver6 = texture_colored.at<cv::Vec3b>(pix_i_ver6, pix_j_ver6);
    int faceID_ver6_pixel = pix_ver6[2] * 256 * 256 + pix_ver6[1] * 256 + pix_ver6[0];
    faceID_ver6_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver7 = texture_colored.at<cv::Vec3b>(pix_i_ver7, pix_j_ver7);
    int faceID_ver7_pixel = pix_ver7[2] * 256 * 256 + pix_ver7[1] * 256 + pix_ver7[0];
    faceID_ver7_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver8 = texture_colored.at<cv::Vec3b>(pix_i_ver8, pix_j_ver8);
    int faceID_ver8_pixel = pix_ver8[2] * 256 * 256 + pix_ver8[1] * 256 + pix_ver8[0];
    faceID_ver8_pixel--; // undo previous increment (-1 means no triangle ID)

    const cv::Vec3b pix_ver9 = texture_colored.at<cv::Vec3b>(pix_i_ver9, pix_j_ver9);
    int faceID_ver9_pixel = pix_ver9[2] * 256 * 256 + pix_ver9[1] * 256 + pix_ver9[0];
    faceID_ver9_pixel--; // undo previous increment (-1 means no triangle ID)

    bool ver1_valid = (faceID_ver1_pixel < 0) ? ver1_valid = false : ver1_valid = true;
    bool ver2_valid = (faceID_ver2_pixel < 0) ? ver2_valid = false : ver2_valid = true;
    bool ver3_valid = (faceID_ver3_pixel < 0) ? ver3_valid = false : ver3_valid = true;
    bool ver4_valid = (faceID_ver4_pixel < 0) ? ver4_valid = false : ver4_valid = true;
    bool ver5_valid = (faceID_ver5_pixel < 0) ? ver5_valid = false : ver5_valid = true;
    bool ver6_valid = (faceID_ver6_pixel < 0) ? ver6_valid = false : ver6_valid = true;
    bool ver7_valid = (faceID_ver7_pixel < 0) ? ver7_valid = false : ver7_valid = true;
    bool ver8_valid = (faceID_ver8_pixel < 0) ? ver8_valid = false : ver8_valid = true;
    bool ver9_valid = (faceID_ver9_pixel < 0) ? ver9_valid = false : ver9_valid = true;

    const double diff_ver1 = std::pow(pix_i - pix_i_ver1, 2) + std::pow(pix_j - pix_j_ver1, 2);
    const double diff_ver2 = std::pow(pix_i - pix_i_ver2, 2) + std::pow(pix_j - pix_j_ver2, 2);
    const double diff_ver3 = std::pow(pix_i - pix_i_ver3, 2) + std::pow(pix_j - pix_j_ver3, 2);
    const double diff_ver4 = std::pow(pix_i - pix_i_ver4, 2) + std::pow(pix_j - pix_j_ver4, 2);
    const double diff_ver5 = std::pow(pix_i - pix_i_ver5, 2) + std::pow(pix_j - pix_j_ver5, 2);
    const double diff_ver6 = std::pow(pix_i - pix_i_ver6, 2) + std::pow(pix_j - pix_j_ver6, 2);
    const double diff_ver7 = std::pow(pix_i - pix_i_ver7, 2) + std::pow(pix_j - pix_j_ver7, 2);
    const double diff_ver8 = std::pow(pix_i - pix_i_ver8, 2) + std::pow(pix_j - pix_j_ver8, 2);
    const double diff_ver9 = std::pow(pix_i - pix_i_ver9, 2) + std::pow(pix_j - pix_j_ver9, 2);

    bool found_valid = false;
    graphene::Vec2ui best_pixel;

    if (ver1_valid || ver2_valid || ver3_valid || ver4_valid || ver5_valid ||
        ver6_valid || ver7_valid || ver8_valid || ver9_valid)
    {
        found_valid = true;

        double best_diff = DBL_MAX;
        if (ver1_valid && diff_ver1 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver1, pix_j_ver1);
            best_diff = diff_ver1;
        }
        if (ver2_valid && diff_ver2 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver2, pix_j_ver2);
            best_diff = diff_ver2;
        }
        if (ver3_valid && diff_ver3 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver3, pix_j_ver3);
            best_diff = diff_ver4;
        }
        if (ver4_valid && diff_ver4 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver4, pix_j_ver4);
            best_diff = diff_ver4;
        }
        if (ver5_valid && diff_ver5 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver5, pix_j_ver5);
            best_diff = diff_ver5;
        }
        if (ver6_valid && diff_ver6 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver6, pix_j_ver6);
            best_diff = diff_ver6;
        }
        if (ver7_valid && diff_ver7 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver7, pix_j_ver7);
            best_diff = diff_ver7;
        }
        if (ver8_valid && diff_ver8 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver8, pix_j_ver8);
            best_diff = diff_ver8;
        }
        if (ver9_valid && diff_ver9 < best_diff)
        {
            best_pixel = graphene::Vec2ui(pix_i_ver9, pix_j_ver9);
            best_diff = diff_ver9;
        }
    }
    else
    {
        found_valid = false;
        best_pixel = graphene::Vec2ui(pix_i_ver1, pix_j_ver1);
    }

    result = best_pixel;

    return found_valid;
}

//=============================================================================
} // namespace utility
} // namespace graphene
//=============================================================================
