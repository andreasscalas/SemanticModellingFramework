//=============================================================================
#ifndef GRAPHENE_TEXTUREPROCESSING_HELPER_H
#define GRAPHENE_TEXTUREPROCESSING_HELPER_H
//=============================================================================

//== INCLUDES ===================================================================

#include <graphene/gl/Texture.h>

//== HELPER ===================================================================


static cv::Mat
texture_to_opencv(graphene::gl::Texture* texture)
{
    int height = texture->height_;
    int width  = texture->width_;
    cv::Mat img(height, width, CV_8UC3);

    unsigned int c_idx = 0;
    for (unsigned int i = 0; i < height; ++i)
    {
        for (unsigned int j = 0; j < width; ++j)
        {
            img.at<cv::Vec3b>(height - i - 1,j)[2] = texture->data_[c_idx++];
            img.at<cv::Vec3b>(height - i - 1,j)[1] = texture->data_[c_idx++];
            img.at<cv::Vec3b>(height - i - 1,j)[0] = texture->data_[c_idx++];
            c_idx++; // skip alpha
        }
    }

    return img;
}


//-----------------------------------------------------------------------------


static cv::Mat
texture_to_opencv(int width, int height, unsigned char* pixels)
{
    cv::Mat img(height, width, CV_8UC3);

    unsigned int c_idx = 0;
    for (unsigned int i = 0; i < height; ++i)
    {
        for (unsigned int j = 0; j < width; ++j)
        {
            img.at<cv::Vec3b>(height - i - 1,j)[2] = pixels[c_idx++];
            img.at<cv::Vec3b>(height - i - 1,j)[1] = pixels[c_idx++];
            img.at<cv::Vec3b>(height - i - 1,j)[0] = pixels[c_idx++];
            c_idx++; // skip alpha
        }
    }

    return img;
}


//-----------------------------------------------------------------------------


static void
opencv_to_texture(cv::Mat& texture_opencv, graphene::gl::Texture* texture)
{
    // TODO CHECK DIMENSIONS
    int height = texture->height_;
    int width  = texture->width_;

    unsigned int c_idx = 0;
    for (unsigned int i = 0; i < height; ++i)
    {
        for (unsigned int j = 0; j < width; ++j)
        {
            texture->data_[c_idx++] = texture_opencv.at<cv::Vec3b>(height - i - 1,j)[2];
            texture->data_[c_idx++] = texture_opencv.at<cv::Vec3b>(height - i - 1,j)[1];
            texture->data_[c_idx++] = texture_opencv.at<cv::Vec3b>(height - i - 1,j)[0];
            c_idx++; // skip alpha
        }
    }
}


//-----------------------------------------------------------------------------


static void
compute_texture_colored_triangle_ids(const graphene::surface_mesh::Surface_mesh& mesh,
                                     cv::Mat& texture_colored)
{
    const unsigned int rows = texture_colored.rows;
    const unsigned int cols = texture_colored.cols;
    if (rows != cols || rows == 0)
    {
        std::cerr << "[ERROR] in 'compute_texture_colored_triangle_ids': rows != cols || rows == 0" << std::endl;
        return;
    }

    const unsigned int dim = rows;

    texture_colored.setTo(cv::Scalar(0));

    auto texcoords = mesh.get_halfedge_property<graphene::Texture_coordinate>("h:texcoord");
    if (!texcoords)
    {
        std::cerr << "[ERROR] no tex coords." << std::endl;
        return;
    }
    for (auto f : mesh.faces())
    {
        // texture coordinates
        std::vector<Eigen::Vector2d> tex_coords;
        for (auto fh : mesh.halfedges(f))
        {
            graphene::Texture_coordinate t = texcoords[fh];
            tex_coords.push_back( Eigen::Vector2d(t[0], t[1]) );
        }

        // corners
        const graphene::Vec2f corner_1 = graphene::Vec2f(tex_coords[0][0], 1.0 - tex_coords[0][1]) * static_cast<double>(dim);
        const graphene::Vec2f corner_2 = graphene::Vec2f(tex_coords[1][0], 1.0 - tex_coords[1][1]) * static_cast<double>(dim);
        const graphene::Vec2f corner_3 = graphene::Vec2f(tex_coords[2][0], 1.0 - tex_coords[2][1]) * static_cast<double>(dim);
        cv::Point corner_points[1][3];
        corner_points[0][0] = cv::Point(corner_1[0], corner_1[1]);
        corner_points[0][1] = cv::Point(corner_2[0], corner_2[1]);
        corner_points[0][2] = cv::Point(corner_3[0], corner_3[1]);

        // map face idx to rgb values
        unsigned char r, g, b;
        size_t temp = f.idx() + 1; // to avoid confusion with the black background
        r = temp / (256 * 256);
        g = (temp % (256 * 256)) / 256;
        b = temp % 256;

        // fill poly
        const cv::Point* ppt[1] = { corner_points[0] };
        int npt[] = { 3 };
        cv::fillPoly(texture_colored, ppt, npt, 1, cv::Scalar(b, g, r), 8);
    }
}


//-----------------------------------------------------------------------------


static cv::Vec3b
bilinear_access(const double y, const double x, const cv::Mat& image)
{
    const int img_width  = image.cols;
    const int img_height = image.rows;

    int y1 = std::floor(y);
    int y2 = std::ceil(y);

    int x1 = std::floor(x);
    int x2 = std::ceil(x);

    y1 = y1 < 0            ? 0           : y1;
    y2 = y2 < 0            ? 0           : y2;
    y1 = y1 > img_height-1 ? img_width-1 : y1;
    y2 = y2 > img_height-1 ? img_width-1 : y2;

    x1 = x1 < 0           ? 0           : x1;
    x2 = x2 < 0           ? 0           : x2;
    x1 = x1 > img_width-1 ? img_width-1 : x1;
    x2 = x2 > img_width-1 ? img_width-1 : x2;

    cv::Vec3b result;

    // case 1: no interpolation
    if (y1 == y2 && x1 == x2)
    {
        result = image.at<cv::Vec3b>(y1, x1);
    }
    // case 2: only linear interpolation in y-dir
    if (y1 != y2 && x1 == x2)
    {
        const cv::Vec3b f11 = image.at<cv::Vec3b>(y1, x1);
        const cv::Vec3b f21 = image.at<cv::Vec3b>(y2, x1);

        result = f11 + (f21-f11)*(y-y1);
    }
    // case 3: only linear interpolation in x-dir
    else if (x1 != x2 && y1 == y2)
    {
        const cv::Vec3b f11 = image.at<cv::Vec3b>(y1, x1);
        const cv::Vec3b f12 = image.at<cv::Vec3b>(y1, x2);

        result = f11 + (f12-f11)*(x-x1);
    }
    // case 4: bilinear interpolation
    else
    {
        const cv::Vec3b f11 = image.at<cv::Vec3b>(y1, x1);
        const cv::Vec3b f21 = image.at<cv::Vec3b>(y2, x1);
        const cv::Vec3b f12 = image.at<cv::Vec3b>(y1, x2);
        const cv::Vec3b f22 = image.at<cv::Vec3b>(y2, x2);

        const cv::Vec3b fh1 = f11 + (f21-f11)*(y-y1);
        const cv::Vec3b fh2 = f12 + (f22-f12)*(y-y1);

        result = fh1 + (fh2-fh1)*(x-x1);
    }

    return result;
}


//-----------------------------------------------------------------------------


static void
compute_texture_triangle_pixels(graphene::surface_mesh::Surface_mesh& mesh,
                                cv::Mat& texture_colors,
                                const unsigned int dim)
{
    texture_colors = cv::Mat(dim, dim, CV_8UC3, cv::Scalar(0));

    compute_texture_colored_triangle_ids(mesh, texture_colors);

    auto texcoords = mesh.get_halfedge_property<graphene::Texture_coordinate>("h:texcoord");
    if (!texcoords)
    {
        std::cerr << "[ERROR] no tex coords." << std::endl;
        return;
    }

    //cv::imwrite("texture_colors.png", texture_colors);
    //cv::namedWindow("texture_colors"); cv::imshow("texture_colors", texture_colors); cv::waitKey(0); cv::destroyWindow("texture_colors");

    auto texture_triangle_pixels = mesh.face_property< std::vector<Eigen::Vector2i> >("f:texpixels");
    auto texture_triangle_barys = mesh.face_property< std::vector<Eigen::Vector3d> >("f:texbarys");

    for (int x = 0; x < texture_colors.cols; ++x)
    {
        for (int y = 0; y < texture_colors.rows; ++y)
        {
            const cv::Vec3b pix = texture_colors.at<cv::Vec3b>(y, x);
            int tri_id = pix[2] * 256 * 256 + pix[1] * 256 + pix[0];

            if (tri_id == 0) // texel corresponds to no triangle
            {
                continue; // nothing to do
            }
            tri_id --; // undo previous increment

            graphene::surface_mesh::Surface_mesh::Face f_handle(tri_id);

            // store texture coordinates column-wise
            Eigen::Matrix3d U;
            int index = 0; // column
            for (auto fh : mesh.halfedges(f_handle))
            {
                graphene::Texture_coordinate t = texcoords[fh];
                for (unsigned int i = 0; i < 3; ++i)
                {
                    U(i, index) = t[i];
                }
                index ++;
            }

            Eigen::Vector3d rhs(       static_cast<double>(x) / (static_cast<double>(dim) - 1.0),
                                1.0 - (static_cast<double>(y) / (static_cast<double>(dim) - 1.0)),
                                1.0);
            Eigen::Vector3d A = U.inverse() * rhs;

            texture_triangle_pixels[f_handle].push_back( Eigen::Vector2i(y, x) );
            texture_triangle_barys[f_handle].push_back( A );
        }
    }
}


//=============================================================================
#endif // GRAPHENE_TEXTUREPROCESSING_HELPER_H
//=============================================================================
