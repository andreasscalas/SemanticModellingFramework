//=============================================================================
#ifndef TEXTURE_GENERATOR_H
#define TEXTURE_GENERATOR_H
//=============================================================================

//== INCLUDES =================================================================


#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
#include <graphene/gl/GL_state.h>
#include <graphene/scene_graph/Scene_graph.h>

#include "opencv2/opencv.hpp"

#include <Eigen/Dense>

#include <unordered_set>


//== CLASS DEFINITION =========================================================


enum Texture_generator_mode { best_view, average };


class
Texture_generator
{
public:

    Texture_generator(const graphene::surface_mesh::Surface_mesh& mesh,      // mesh that is located and oriented as in photoscan
                      const std::string& filename_cameras,                   // camera filename
                      const std::string& dirname_photos,                     // dirname of undistorted photos
                      graphene::gl::GL_state* gl_state,                      // needed for visibility computation of triangles
                      const graphene::scene_graph::Scene_graph* scene_graph, // needed to draw the scene during visibility computations
                      const unsigned int dim,                                // dimension of texture to generate
                      const Texture_generator_mode mode,                     // mode
                      cv::Mat& texture_generated);                           // output: generated texture

private:

    std::vector< cv::Vec3b > collect_neighboring_colors(const unsigned int y, const unsigned int x, const cv::Mat& image);


private:

    unsigned int dim_;

    cv::Mat texture_colors_;

    const bool verbose_ = false;

};


//=============================================================================
#endif // TEXTURE_GENERATOR_H
//=============================================================================
