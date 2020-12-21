//=============================================================================
// Copyright (C) 2014 Graphics & Geometry Group, Bielefeld University
//=============================================================================


#include <graphene/surface_mesh/algorithms/surface_mesh_tools/misc.h>

#include <Eigen/Dense>

#include <cfloat>
#include <fstream>


//=============================================================================


namespace graphene {
namespace surface_mesh {


//=============================================================================


Scalar
mean_edge_length(const Surface_mesh& mesh)
{
    Scalar mean = 0;

    for (auto e : mesh.edges())
    {
        mean += mesh.edge_length(e);
    }

    mean /= (Scalar)mesh.n_edges();

    return mean;
}


//-----------------------------------------------------------------------------


Point
centroid(Surface_mesh& mesh, const Surface_mesh::Face& f)
{
    auto points = mesh.vertex_property<Point>("v:point");

    Point centroid(0);
    size_t n_vertices(0);

    for (auto v : mesh.vertices(f))
    {
        centroid += points[v];
        n_vertices++;
    }

    centroid /= (Scalar)n_vertices;

    return centroid;
}


//-----------------------------------------------------------------------------


bool
load_selection_from_file(std::vector<int>& output, const std::string& filename)
{
    std::ifstream ifs(filename);
    if( ifs.is_open() == true )
    {
        output.clear();

        int idx;
        while (true)
        {
            ifs >> idx;
            if (ifs.eof()) break;
            output.push_back(idx);
        }

        ifs.close();

        return ! output.empty();
    }
    else
    {
        std::cerr << "[ERROR] Cannot read selection from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
load_selection_from_file(std::vector<unsigned int>& output, const std::string& filename)
{
    std::ifstream ifs(filename);
    if( ifs.is_open() == true )
    {
        output.clear();

        unsigned int idx;
        while (true)
        {
            ifs >> idx;
            if (ifs.eof()) break;
            output.push_back(idx);
        }
        ifs.close();

        return ! output.empty();
    }
    else
    {
        std::cerr << "[ERROR] Cannot read selection from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
load_weights_from_file(std::vector<double>& output, const std::string& filename)
{
    std::ifstream ifs(filename);
    if( ifs.is_open() == true )
    {
        output.clear();

        double val;
        while (true)
        {
            ifs >> val;
            if (ifs.eof()) break;
            output.push_back(val);
        }

        ifs.close();

        return ! output.empty();
    }
    else
    {
        std::cerr << "[ERROR] Cannot read weights from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
load_selection_from_file(std::vector<size_t>& output, const std::string& filename)
{
    std::ifstream ifs(filename);
    if( ifs.is_open() == true )
    {
        output.clear();

        size_t idx;
        while (true)
        {
            ifs >> idx;
            if (ifs.eof()) break;
            output.push_back(idx);
        }

        ifs.close();

        return ! output.empty();
    }
    else
    {
        std::cerr << "[ERROR] Cannot read selection from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
load_selection_from_file(std::vector<Surface_mesh::Vertex>& output, const std::string& filename)
{
    std::ifstream ifs(filename);
    if( ifs.is_open() == true )
    {
        output.clear();

        unsigned int idx;
        while (true)
        {
            ifs >> idx;
            if (ifs.eof()) break;
            output.push_back(Surface_mesh::Vertex(idx));
        }

        ifs.close();

        return ! output.empty();
    }
    else
    {
        std::cerr << "[ERROR] Cannot read selection from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
load_selection_from_file_with_weights(std::vector<unsigned int>& indices,
                                      std::vector<double>&       local_weights,
                                      const std::string&         filename)
{
    std::ifstream ifs(filename);
    if( ifs.is_open() == true )
    {
        indices.clear();
        local_weights.clear();

        unsigned int idx;
        double       weight;
        while (true)
        {
            ifs >> idx;
            ifs >> weight;
            if (ifs.eof()) break;
            indices.push_back(idx);
            local_weights.push_back(weight);
        }

        ifs.close();

        return ! indices.empty();
    }
    else
    {
        std::cerr << "[ERROR] Cannot read selection with weights from \"" << filename << "\"" << std::endl;
        return false;
    }
}


//-----------------------------------------------------------------------------


bool
intersect_ray_triangle( const Point& tri_p0, const Point& tri_p1, const Point& tri_p2,
                        const Normal& tri_n,
                        const Point& ray_center, const Vec3d& ray_direction,
                        const bool allow_negative_t,
                        Point&  p_intersect,
                        Point&  n_intersect,
                        double& t_intersect )
{
    // solve ray.origin + t*ray.dir = a*p0 + b*p1 + (1-a-b)*p2
    // rearrange to get a 3x3 system A * x = b
    // solve it using Cramer's rule
    // columns of the matrix
    Vec3d a1 = - ray_direction;
    Vec3d a2 = tri_p0 - tri_p2;
    Vec3d a3 = tri_p1 - tri_p2;

    // right hand side of linear system
    Vec3d  b = Vec3d(ray_center) - Vec3d( tri_p2 );

#define det(a,b,c) (dot((cross((a),(b))),(c)))

    const double denom = det(a1, a2, a3);
    if (fabs(denom) <= DBL_MIN)
    {
        return false;
    }

    const double alpha = det(a1, b, a3) / denom;
    if (alpha < 0.0 || alpha > 1.0)
    {
        return false;
    }

    const double beta = det(a1, a2, b) / denom;
    if (beta < 0.0 || alpha+beta > 1.0)
    {
        return false;
    }

    const double gamma = 1.0 - alpha - beta;
    double t = det(b, a2, a3) / denom;
    if (!allow_negative_t && t < 1e-5)
    {
        return false;
    }

#undef det

    p_intersect = ray_center + t * Vec3f(ray_direction);
    t_intersect = t;
    n_intersect = tri_n;

    return true;
}


//-----------------------------------------------------------------------------


bool
intersect_ray_triangle( const Point& tri_p0, const Point& tri_p1, const Point& tri_p2,
                        const Point& ray_center, const Vec3d& ray_direction,
                        const bool allow_negative_t,
                        Point&  p_intersect,
                        double& t_intersect )
{
    Normal tri_n;       // dummy
    Point  n_intersect; // dummy

    return intersect_ray_triangle( tri_p0, tri_p1, tri_p2, tri_n,
                                   ray_center, ray_direction,
                                   allow_negative_t,
                                   p_intersect,
                                   n_intersect,
                                   t_intersect );
}


//-----------------------------------------------------------------------------


bool
intersect_ray_plane( const Point& plane_p0,
                     const Normal& plane_n0,
                     const Point& ray_center, const Vec3d& ray_direction,
                     Point&  p_intersect,
                     double& t_intersect )
{
    const double dn = dot(ray_direction, Vec3d(plane_n0));

    if (fabs(dn) > FLT_MIN)
    {
        const double t = dot(plane_n0, plane_p0 - ray_center) / dn;
        if (t > 1e-5)
        {
            p_intersect = Point(ray_center) + t * Vec3f(ray_direction);
            t_intersect  = t;

            return true;
        }
    }

    return false;
}


//-----------------------------------------------------------------------------


bool
write_eigen_matrix_as_bin(const Eigen::MatrixXd& eigen_matrix,
                          const std::string& filename,
                          const unsigned int rows,
                          const unsigned int cols)
{
    std::ofstream ofs( filename.c_str(), std::ofstream::binary );
    if (!ofs.is_open())
    {
        std::cerr << "[ERROR] in write_eigen_matrix_as_bin(...) - !ofs.is_open()" << std::endl;
        return false;
    }
    unsigned int n_rows = rows;
    unsigned int n_cols = cols;
    ofs.write(reinterpret_cast<const char *>(&n_rows), sizeof(n_rows));
    ofs.write(reinterpret_cast<const char *>(&n_cols), sizeof(n_cols));
    ofs.write(reinterpret_cast<const char *>(eigen_matrix.data()), n_rows*n_cols*sizeof(typename Eigen::MatrixXd::Scalar) );
    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
write_eigen_vector_as_bin(const Eigen::VectorXd& eigen_vector,
                          const std::string& filename,
                          const unsigned int dim)
{
    std::ofstream ofs( filename.c_str(), std::ofstream::binary );
    if (!ofs.is_open())
    {
        std::cerr << "[ERROR] in write_eigen_vector_as_bin(...) - !ofs.is_open()" << std::endl;
        return false;
    }
    unsigned int n_rows = dim;
    ofs.write(reinterpret_cast<const char *>(&n_rows), sizeof(dim));
    ofs.write(reinterpret_cast<const char *>(eigen_vector.data()), n_rows*sizeof(typename Eigen::VectorXd::Scalar) );
    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
read_eigen_matrix_from_bin(Eigen::MatrixXd& eigen_matrix,
                           const std::string& filename)
{
    std::ifstream ifs( filename.c_str(), std::ofstream::binary );
    if (!ifs.is_open())
    {
        std::cerr << "[ERROR] in read_eigen_matrix_from_bin(...) - !ifs.is_open()" << std::endl;
        return false;
    }
    unsigned int n_rows = 0;
    unsigned int n_cols = 0;
    ifs.read(reinterpret_cast<char *>(&n_rows), sizeof(n_rows));
    ifs.read(reinterpret_cast<char *>(&n_cols), sizeof(n_cols));
    eigen_matrix.resize(n_rows, n_cols);
    ifs.read(reinterpret_cast<char *>(eigen_matrix.data()), n_rows*n_cols*sizeof(typename Eigen::MatrixXd::Scalar) );
    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
read_eigen_vector_from_bin(Eigen::VectorXd& eigen_vector,
                           const std::string& filename)
{
    std::ifstream ifs( filename.c_str(), std::ofstream::binary );
    if (!ifs.is_open())
    {
        std::cerr << "[ERROR] in read_eigen_vector_from_bin(...) - !ifs.is_open()" << std::endl;
        return false;
    }
    unsigned int rows = 0;
    ifs.read(reinterpret_cast<char *>(&rows), sizeof(rows));
    eigen_vector = Eigen::VectorXd(rows);
    ifs.read(reinterpret_cast<char *>(eigen_vector.data()), rows*sizeof(typename Eigen::VectorXd::Scalar) );
    ifs.close();

    return true;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
