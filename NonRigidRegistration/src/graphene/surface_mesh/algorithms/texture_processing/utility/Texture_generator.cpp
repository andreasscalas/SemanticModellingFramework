//== INCLUDES ===================================================================


#include "Texture_generator.h"
#include "my_helper.h"

#include <graphene/geometry/bary_coord.h>

#include <graphene/surface_mesh/scene_graph/Surface_mesh_node.h>

#include <graphene/surface_mesh/algorithms/headtemplatefittingtools/utility/Agi_camera.h>

#include <unordered_map>



//== IMPLEMENTATION ============================================================


Texture_generator::
Texture_generator(const graphene::surface_mesh::Surface_mesh& mesh,
                  const std::string& filename_cameras,
                  const std::string& dirname_photos,
                  graphene::gl::GL_state* gl_state,
                  const graphene::scene_graph::Scene_graph* scene_graph,
                  const unsigned int dim,
                  const Texture_generator_mode mode,
                  cv::Mat& texture_generated)
  : dim_(dim)
{
    // load agi_cameras from file
    graphene::utility::Agi_cameras agi_cameras(filename_cameras);

    // load undistorted photos
    std::unordered_map<std::string, cv::Mat> undistorted_photos;
    {
        std::vector<std::string> labels_long = agi_cameras.get_labels_long();
        for (unsigned int i = 0; i < labels_long.size(); ++i)
        {
            std::string filename_current_img = dirname_photos; filename_current_img.append("/"); filename_current_img.append(labels_long[i]);

            cv::Mat current_image = cv::imread(filename_current_img.c_str());
            if(! current_image.data )
            {
                std::cerr << "[ERROR] Can't read image " << filename_current_img << std::endl;
                return;
            }

            undistorted_photos[labels_long[i]] = current_image;
        }
    }

    // encapsulate mesh within a Surface_mesh_node
    graphene::scene_graph::Surface_mesh_node* mesh_node = new graphene::scene_graph::Surface_mesh_node();
    mesh_node->mesh() = mesh;

    // for each triangle, compute pixel coordinates and barycentric coordinates inside that triangle
    if (!mesh_node->mesh().is_triangle_mesh())
    {
        std::cerr << "[ERROR] Mesh is NOT triangular." << std::endl;
        return;
    }
    compute_texture_triangle_pixels(mesh_node->mesh(), texture_colors_, dim_);
    auto texture_triangle_pixels = mesh_node->mesh().get_face_property< std::vector<Eigen::Vector2i> >("f:texpixels");
    auto texture_triangle_barys  = mesh_node->mesh().get_face_property< std::vector<Eigen::Vector3d> >("f:texbarys");
    if (!texture_triangle_pixels || !texture_triangle_barys)
    {
        std::cerr << "[ERROR] No pixels or bary-coords within triangles!" << std::endl;
        return;
    }

    // create new texture
    unsigned char* data = new unsigned char[dim_*dim_*4];
    graphene::gl::Texture* texture = new graphene::gl::Texture("texture_test", dim_, dim_, data);
    texture->srgb_ = false;
    texture->filter_ = GL_NEAREST;
    opencv_to_texture(texture_colors_, texture);
    mesh_node->set_texture(texture, graphene::gl::TT_COLOR);

    // set draw mode
    mesh_node->set_draw_mode("Textured");

    mesh_node->update_mesh();

    // final output textures
    cv::Mat texture_best_view;
    cv::Mat texture_average;
    if (mode == best_view)
    {
        texture_best_view = cv::Mat(dim_, dim_, CV_8UC3, cv::Scalar(0));
    }
    else if (mode == average)
    {
        texture_average   = cv::Mat(dim_, dim_, CV_8UC3, cv::Scalar(0));
    }

    // weights for texels
    Eigen::MatrixXd best_view_angles;
    Eigen::MatrixXd average_weights;
    std::vector<Eigen::Vector3d> texture_average_vec;
    if (mode == best_view)
    {
        best_view_angles = Eigen::MatrixXd(dim_, dim_); best_view_angles.setZero();
    }
    else if (mode == average)
    {
        average_weights = Eigen::MatrixXd(dim_, dim_); average_weights.setZero();
        texture_average_vec.resize(dim_ * dim_);
        for (unsigned int i = 0; i < dim_ * dim_; ++i)
        {
            texture_average_vec[i] = Eigen::Vector3d::Zero();
        }
    }

    auto normals = mesh_node->mesh().get_vertex_property<graphene::Normal>("v:normal");
    if (!normals)
    {
        std::cerr << "[ERROR] No normals available!" << std::endl;
        return;
    }

    std::vector< cv::Mat > visible_texels_vec;
    std::vector< cv::Mat > image_tex_vec;
    graphene::gl::Framebuffer* fb_render_to_texture = 0;
    for (size_t cam_i = 0; cam_i < agi_cameras.get_cameras().size(); ++cam_i)
    {
        std::cerr << "[DEBUG] cam_i: " << cam_i << std::endl;

        const graphene::utility::Agi_camera& agi_camera = agi_cameras.get_cameras()[cam_i];

        // compute which texels correspond to visible 3d points
        std::cerr << "[DEBUG] compute which texels correspond to visible 3d points" << std::endl;
        cv::Mat visible_texels(dim_, dim_, CV_8UC1, cv::Scalar(0));
        {
            // render image            
            graphene::Mat4d mat_Rt = agi_camera.get_mat_Rt();
            graphene::Mat4d mat_K  = agi_camera.get_mat_K();

            double fx   = mat_K(0,0);
            double fy   = mat_K(1,1);

            const float near1 = 0.1;
            const float far1  = 100.0;

            const int img_width  = agi_camera.get_width();
            const int img_height = agi_camera.get_height();

            // new modelview
            const graphene::Vec3d eye(0.0, 0.0, 0.0);
            const graphene::Vec3d center(0.0, 0.0, 1.0);
            const graphene::Vec3d up(0.0, -1.0, 0.0);
            graphene::Mat4d new_modelview = graphene::Mat4d::look_at(eye, center, up) * inverse(mat_Rt);

            // new projection
            double fovy = 2*atan(0.5*img_height/fy)*180/M_PI;
            double aspect = (img_width*fy)/(img_height*fx);
            graphene::Mat4f new_projection = graphene::Mat4f::perspective(fovy, aspect, near1, far1);

            // set MVP
            gl_state->model_ = graphene::Mat4f::identity();
            gl_state->view_  = graphene::Mat4f(new_modelview);
            gl_state->proj_  = graphene::Mat4f(new_projection);

            if (!fb_render_to_texture)
            {
                fb_render_to_texture = new graphene::gl::Framebuffer_singlesample(img_width, img_height);
            }
            else
            {
                fb_render_to_texture->resize(img_width, img_height);
            }

            fb_render_to_texture->bind();
            glViewport(0, 0, img_width, img_height);

            // draw to texture
            gl_state->update_matrices();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            mesh_node->draw(gl_state);

            glBindTexture(GL_TEXTURE_2D, fb_render_to_texture->get_color_attachment()->texture_id_);

            unsigned char* pixels = new unsigned char[img_width*img_height*4];

            glGetTexImage( GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

            cv::Mat texture_rendered = texture_to_opencv(img_width, img_height, pixels);

            if (verbose_)
            {
                cv::imwrite("texture_rendered_" + agi_camera.get_label_long() /*std::to_string(cam_i)*/ + ".png", texture_rendered);
                // cv::namedWindow("texture_rendered"); cv::imshow("texture_rendered", texture_rendered); cv::waitKey(0); cv::destroyWindow("texture_rendered");
            }

            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            delete pixels;

            // visibility for texels: compare texel color from 'texture_colors_' with
            //                        projected 3d point in colored rendering (for all triangles)
            for (auto f : mesh_node->mesh().faces())
            {
                for (unsigned int pix_i = 0; pix_i < texture_triangle_pixels[f].size(); ++pix_i)
                {
                    // get current texel color from 'texture_colors_'

                    const int tex_tri_pix_y = texture_triangle_pixels[f][pix_i](0);
                    const int tex_tri_pix_x = texture_triangle_pixels[f][pix_i](1);

                    cv::Vec3b texel_color = texture_colors_.at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x );

                    // get color from projected 3d point in rendering
                    Eigen::Vector3d vpos( Eigen::Vector3d::Zero() );
                    Eigen::Vector3d vnormal( Eigen::Vector3d::Zero() );
                    int index = 0;
                    for (auto fv : mesh_node->mesh().vertices(f))
                    {
                        graphene::Point v = mesh_node->mesh().position(fv);
                        vpos += Eigen::Vector3d(v[0], v[1], v[2]) * texture_triangle_barys[f][pix_i](index);

                        graphene::Normal n = normals[fv];
                        vnormal += Eigen::Vector3d(n[0], n[1], n[2]) * texture_triangle_barys[f][pix_i](index);

                        index ++;
                    }
                    vnormal.normalize();

                    const graphene::Vec3f dir_point_to_camera = ( agi_camera.get_center() - graphene::Point(vpos(0), vpos(1), vpos(2)) ).normalize();
                    const double angle_cos = graphene::dot( graphene::Normal(vnormal(0), vnormal(1), vnormal(2)) , dir_point_to_camera );
                    if (angle_cos <= 0.0)
                    {
                        continue;
                    }

                    // projection
                    const graphene::Mat4f new_MVP = new_projection*graphene::Mat4f(new_modelview);
                    const graphene::Vec4f vpos_ogl(vpos(0), vpos(1), vpos(2), 1.0);
                    graphene::Vec4f v_proj_ogl = new_MVP * vpos_ogl;
                    v_proj_ogl /= v_proj_ogl[3];
                    v_proj_ogl *= graphene::Vec4f(0.5f);
                    v_proj_ogl += graphene::Vec4f(0.5f);
                    v_proj_ogl[0] *= img_width;
                    v_proj_ogl[1] = img_height - v_proj_ogl[1] * img_height;
                    const double v_proj_x = v_proj_ogl[0];
                    const double v_proj_y = v_proj_ogl[1];

                    // check pixel range
                    if (v_proj_x < 0 || v_proj_x >= agi_camera.get_width() ||
                        v_proj_y < 0 || v_proj_y >= agi_camera.get_height())
                    {
                        continue;
                    }

                    // check if is inside neighborhood, due to rounding errors
                    std::vector< cv::Vec3b > neighbors = collect_neighboring_colors(std::round(v_proj_y), std::round(v_proj_x), texture_rendered);
                    for (auto n : neighbors)
                    {
                        if ( n == texel_color )
                        {
                            visible_texels.at<uchar>(tex_tri_pix_y, tex_tri_pix_x) = 255;
                            break;
                        }
                    }
                }
            }
        }

        // closing on visibility texture
        cv::Mat closing_element(2, 2, CV_8U, cv::Scalar(255));
        cv::morphologyEx(visible_texels, visible_texels, cv::MORPH_CLOSE, closing_element);

        if (verbose_)
        {
            std::cerr << "[DEBUG] writing visibility texture." << std::endl;
            cv::imwrite("visible_texels_" + agi_camera.get_label_long() /*std::to_string(cam_i)*/ + ".png", visible_texels);
        }

        visible_texels_vec.push_back(visible_texels);

        cv::Mat image_tex(dim_, dim_, CV_8UC3, cv::Scalar(0));
        for (auto f : mesh_node->mesh().faces())
        {
            for (unsigned int pix_i = 0; pix_i < texture_triangle_pixels[f].size(); ++pix_i)
            {
                const int tex_tri_pix_y = texture_triangle_pixels[f][pix_i](0);
                const int tex_tri_pix_x = texture_triangle_pixels[f][pix_i](1);

                if (visible_texels.at<uchar>(tex_tri_pix_y, tex_tri_pix_x) == 0)
                {
                    continue;
                }

                // get the position and normal for the surface point that projects onto this texel
                Eigen::Vector3d vpos( Eigen::Vector3d::Zero() );
                Eigen::Vector3d vnormal( Eigen::Vector3d::Zero() );

                int index = 0;
                for (auto fv : mesh_node->mesh().vertices(f))
                {
                    graphene::Point v = mesh_node->mesh().position(fv);
                    vpos += Eigen::Vector3d(v[0], v[1], v[2]) * texture_triangle_barys[f][pix_i](index);

                    graphene::Normal n = normals[fv];
                    vnormal += Eigen::Vector3d(n[0], n[1], n[2]) * texture_triangle_barys[f][pix_i](index);

                    index ++;
                }
                vnormal.normalize();

                const graphene::Vec3f dir_point_to_camera = ( agi_camera.get_center() - graphene::Point(vpos(0), vpos(1), vpos(2)) ).normalize();
                const double angle_cos = graphene::dot( graphene::Normal(vnormal(0), vnormal(1), vnormal(2)) , dir_point_to_camera );
                if (angle_cos <= 0.0)
                {
                    continue;
                }

                graphene::Vec2d v_proj = agi_camera.world_to_pixel( graphene::Point(vpos(0), vpos(1), vpos(2)) );
                double v_proj_x = v_proj[0];
                double v_proj_y = v_proj[1];

                // check pixel range
                if (v_proj_x < 0 || v_proj_x >= agi_camera.get_width() ||
                    v_proj_y < 0 || v_proj_y >= agi_camera.get_height())
                {
                    continue;
                }

                cv::Mat& current_image = undistorted_photos[ agi_camera.get_label_long() ];
                const cv::Vec3b color = bilinear_access(v_proj_y, v_proj_x, current_image);

                image_tex.at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x ) = color;
                if (mode == best_view)
                {
                    // best view selection
                    if (best_view_angles(tex_tri_pix_y, tex_tri_pix_x) < angle_cos)
                    {
                        best_view_angles(tex_tri_pix_y, tex_tri_pix_x) = angle_cos;
                        texture_best_view.at<cv::Vec3b>(tex_tri_pix_y, tex_tri_pix_x) = color;
                    }
                }
                else if (mode == average)
                {
                    if (angle_cos > std::cos(70.0 * M_PI / 180.0)) // i.e., diff < 70 degree
                    {
                        Eigen::Vector3d color_f(color[0], color[1], color[2]);

                        const int vector_index = tex_tri_pix_y*dim_ + tex_tri_pix_x;
                        texture_average_vec[vector_index] = (average_weights(tex_tri_pix_y, tex_tri_pix_x) * texture_average_vec[vector_index] + color_f * angle_cos) / (average_weights(tex_tri_pix_y, tex_tri_pix_x) + angle_cos);

                        average_weights(tex_tri_pix_y, tex_tri_pix_x) += angle_cos;
                    }
                }
            }
        }

        if (verbose_)
        {
            std::cerr << "[DEBUG] writing current texture." << std::endl;
            cv::imwrite("projected_image_to_texture_" + agi_camera.get_label_long() /*std::to_string(cam_i)*/ + ".png", image_tex);
        }

        image_tex_vec.push_back( image_tex );
    }

    if (mode == best_view)
    {
        texture_generated = texture_best_view.clone();
    }
    else if (mode == average)
    {
        for (int y = 0; y < dim_; ++y)
        {
            for (int x = 0; x < dim_; ++x)
            {
                const int vector_index = y * dim_ + x;

                cv::Vec3b color(texture_average_vec[vector_index](0),
                                texture_average_vec[vector_index](1),
                                texture_average_vec[vector_index](2));

                texture_average.at<cv::Vec3b>( y, x ) = color;
            }
        }

        texture_generated = texture_average.clone();
    }

    // photometric consistency
    cv::Mat photometric_consistency(dim_, dim_, CV_8UC1, cv::Scalar(255));
    for (auto f : mesh_node->mesh().faces())
    {
        for (unsigned int pix_i = 0; pix_i < texture_triangle_pixels[f].size(); ++pix_i)
        {
            const int tex_tri_pix_y = texture_triangle_pixels[f][pix_i](0);
            const int tex_tri_pix_x = texture_triangle_pixels[f][pix_i](1);

            // mean
            double mean_x = 0;
            double mean_y = 0;
            double mean_z = 0;
            unsigned int num_visible = 0;
            for (size_t cam_i = 0; cam_i < agi_cameras.get_cameras().size(); ++cam_i)
            {
                // only visible points/texels contribute to photometric consistency
                if ( visible_texels_vec[cam_i].at<uchar>( tex_tri_pix_y, tex_tri_pix_x ) > 0 )
                {
                    mean_x += image_tex_vec[cam_i].at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x )[0];
                    mean_y += image_tex_vec[cam_i].at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x )[1];
                    mean_z += image_tex_vec[cam_i].at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x )[2];
                    num_visible++;
                }
            }
            if (num_visible > 0)
            {
                mean_x /= num_visible;
                mean_y /= num_visible;
                mean_z /= num_visible;
            }
            else
            {
                photometric_consistency.at<uchar>( tex_tri_pix_y, tex_tri_pix_x ) = 0;
                continue;
            }

            // standard deviation (photometric consistency is measured by the standard deviation of the color distribution)
            double std_dev_x = 0;
            double std_dev_y = 0;
            double std_dev_z = 0;
            for (size_t cam_i = 0; cam_i < agi_cameras.get_cameras().size(); ++cam_i)
            {
                // only visible points/texels contribute to photometric consistency
                if ( visible_texels_vec[cam_i].at<uchar>( tex_tri_pix_y, tex_tri_pix_x ) > 0 )
                {                
                    const double diff_x = image_tex_vec[cam_i].at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x )[0] - mean_x;
                    std_dev_x += diff_x*diff_x;

                    const double diff_y = image_tex_vec[cam_i].at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x )[1] - mean_y;
                    std_dev_y += diff_y*diff_y;

                    const double diff_z = image_tex_vec[cam_i].at<cv::Vec3b>( tex_tri_pix_y, tex_tri_pix_x )[2] - mean_z;
                    std_dev_z += diff_z*diff_z;
                }
            }
            std_dev_x /= num_visible;
            std_dev_y /= num_visible;
            std_dev_z /= num_visible;
            std_dev_x = std::sqrt(std_dev_x);
            std_dev_y = std::sqrt(std_dev_y);
            std_dev_z = std::sqrt(std_dev_z);

            // if not consistent, set to zero
            const double std_dev = sqrt( std_dev_x*std_dev_x + std_dev_y*std_dev_y + std_dev_z*std_dev_z );
            if (std_dev > 70)
            {
                photometric_consistency.at<uchar>( tex_tri_pix_y, tex_tri_pix_x ) = 0;
            }            
        }
    }
    /*
    for (unsigned int i = 0; i < dim_; ++i)
    {
        for (unsigned int j = 0; j < dim_; ++j)
        {
            if ( photometric_consistency.at<uchar>( i, j ) == 0 )
            {
                texture_generated.at<cv::Vec3b>( i, j ) = 0;
            }
        }
    }
    */

    // visibility
    cv::Mat visibility_total(dim_, dim_, CV_8UC1, cv::Scalar(255));
    for (auto f : mesh_node->mesh().faces())
    {
        for (unsigned int pix_i = 0; pix_i < texture_triangle_pixels[f].size(); ++pix_i)
        {
            const int tex_tri_pix_y = texture_triangle_pixels[f][pix_i](0);
            const int tex_tri_pix_x = texture_triangle_pixels[f][pix_i](1);

            bool visible = false;
            for (size_t cam_i = 0; cam_i < agi_cameras.get_cameras().size(); ++cam_i)
            {
                if ( visible_texels_vec[cam_i].at<uchar>( tex_tri_pix_y, tex_tri_pix_x ) > 0 )
                {
                    visible = true;
                    break;
                }
            }

            // if not visible, set to zero
            if (!visible)
            {
                visibility_total.at<uchar>( tex_tri_pix_y, tex_tri_pix_x ) = 0;
            }            
        }
    }

    if (verbose_)
    {
        cv::imwrite("photometric_consistency.png", photometric_consistency);
        // cv::namedWindow("photometric_consistency"); cv::imshow("photometric_consistency", photometric_consistency); cv::waitKey(0); cv::destroyWindow("photometric_consistency");

        cv::imwrite("texture_generated.png", texture_generated);
        // cv::namedWindow("texture_generated"); cv::imshow("texture_generated", texture_generated); cv::waitKey(0); cv::destroyWindow("texture_generated");

        cv::imwrite("visibility_total.png", visibility_total);
        // cv::namedWindow("visibility_total"); cv::imshow("visibility_total", visibility_total); cv::waitKey(0); cv::destroyWindow("visibility_total");
    }

    // clean up
    mesh_node->mesh().remove_face_property(texture_triangle_pixels);
    mesh_node->mesh().remove_face_property(texture_triangle_barys);
    if (mesh_node)
    {
        delete mesh_node;
    }
    if (fb_render_to_texture)
    {
        delete fb_render_to_texture;
    }
}


//------------------------------------------------------------------------------


std::vector< cv::Vec3b >
Texture_generator::
collect_neighboring_colors(const unsigned int y, const unsigned int x, const cv::Mat& image)
{
    std::vector< cv::Vec3b > neighbors;

    const int img_width  = image.cols;
    const int img_height = image.rows;

    const int range = 1;
    for (int i = y - range; i <= y + range; ++i)
    {
        for (int j = x - range; j <= x + range; ++j)
        {
            unsigned int my_j = j < 0            ? 0            : j;
                         my_j = j > img_width-1  ? img_width-1  : j;
            unsigned int my_i = i < 0            ? 0            : i;
                         my_i = i > img_height-1 ? img_height-1 : i;

            neighbors.push_back( image.at<cv::Vec3b>(my_i, my_j) );
        }
    }

    return neighbors;
}


//==============================================================================
