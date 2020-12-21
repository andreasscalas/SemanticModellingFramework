//=============================================================================
#ifndef GRAPHENE_PCA_MYHELPER_H
#define GRAPHENE_PCA_MYHELPER_H
//=============================================================================

//== INCLUDES ===================================================================

// TODO


//== HELPER ===================================================================
namespace graphene
{
namespace surface_mesh
{


static bool
check_dimension_PCA( const PCA& pca, const unsigned int nv )
{
    // check P matrix and m vector
    if (pca.P_pca_.rows() != 3*nv               ||
        pca.P_pca_.cols() != pca.dim_pca_model_ ||
        pca.m_pca_.size() != 3*nv)
    {
        std::cerr << "check_dimension_PCA(): [ERROR] Wrong dimension of PCA model. P_pca_.rows(): " << pca.P_pca_.rows()
                  << ". 3*nv: " << 3*nv
                  << ". P_pca_.cols(): " << pca.P_pca_.cols()
                  << ". dim_pca_model_: " << pca.dim_pca_model_
                  << ". m_pca_.size(): " << pca.m_pca_.size()
                  << ". Aborting..." << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}


//-----------------------------------------------------------------------------


static bool
train_pca_model_from_meshes(PCA& pca,
                const std::vector<Surface_mesh*>& training_meshes,
                const unsigned int num_components)
{
    if (training_meshes.empty())
    {
        std::cout << "train_pca_model: [ERROR] No training meshes. Cannot perform PCA." << std::endl;
        return false;
    }

    const unsigned int dim_of_training_mesh = 3 * training_meshes.back()->n_vertices(); // xyz-coordinates
    const unsigned int number_of_training_meshes = training_meshes.size();

    if ( num_components > number_of_training_meshes )
    {
        std::cout << "train_pca_model: [ERROR] #Components > #Training meshes. Cannot perform PCA." << std::endl;
        return false;
    }

    // fill data matrix
    Eigen::MatrixXd data(dim_of_training_mesh, number_of_training_meshes);
    for (unsigned int j = 0; j < number_of_training_meshes; ++j)
    {
        // get (pre-defined) property storing vertex positions
        const auto  points = training_meshes[j]->vertex_property<Point>("v:point");

        unsigned int i = 0;
        for (auto v : training_meshes[j]->vertices())
        {
            // access point property like an array
            Point p = points[v];

            data(3*i + 0, j) = p[0];
            data(3*i + 1, j) = p[1];
            data(3*i + 2, j) = p[2];

            ++i;
        }
    }

    return pca.train(data, num_components);
}


//-----------------------------------------------------------------------------


static void
create_mean_mesh(Surface_mesh& mesh,
                 const std::vector<Surface_mesh*>& training_meshes,
                 const PCA& pca)
{
    if (training_meshes.empty())
    {
        return;
    }

    mesh = *training_meshes[0];

    auto mean_points = mesh.get_vertex_property<Point>("v:point");

    unsigned int i = 0;
    for (auto v : mesh.vertices())
    {
        mean_points[v][0] = pca.m_pca_(3*i + 0);
        mean_points[v][1] = pca.m_pca_(3*i + 1);
        mean_points[v][2] = pca.m_pca_(3*i + 2);

        ++i;
    }

    mesh.update_face_normals();
    mesh.update_vertex_normals();
}


//-----------------------------------------------------------------------------


/* TODO
static void
project_mesh(Surface_mesh& mesh, const PCA& pca)
{
    // get (pre-defined) property storing vertex positions
    auto  points = mesh.vertex_property<Point>("v:point");

    // project mesh to PCA space
    std::cout << "projecting mesh into PCA space..." << std::endl;
    const int dim_of_training_mesh = pca_ocv_.pca_.eigenvectors.cols;
    Eigen::VectorXd testmodel(dim_of_training_mesh);
    unsigned int i = 0;
    for (auto v : mesh.vertices())
    {
        // access point property like an array
        const Point p = points[v];
        testmodel(3*i + 0) = p[0];
        testmodel(3*i + 1) = p[1];
        testmodel(3*i + 2) = p[2];
        ++i;
    }
    Eigen::VectorXd res_coeffs;
    pca_ocv_.project(testmodel, res_coeffs);
    // std::cout << "res_coeffs: " << res_coeffs << std::endl;
    // std::cout << "# components of res_coeffs: " << res_coeffs.size() << std::endl;
    // reproject/reconstruct from PCA space
    std::cout << "reprojecting/reconstructing from PCA space..." << std::endl;
    Eigen::VectorXd back;
    pca_ocv_.back_project(res_coeffs, back);
    // std::cout << "# components of back: " << back.size() << std::endl;
    // update vertex positions of mesh
    i = 0;
    for (auto v : mesh.vertices())
    {
        points[v][0] = back(3*i + 0);
        points[v][1] = back(3*i + 1);
        points[v][2] = back(3*i + 2);

        ++i;
    }

    mesh.update_face_normals();
    mesh.update_vertex_normals();
}
*/


} //namespace surface_mesh
} //namespace graphene
//=============================================================================
#endif // GRAPHENE_PCA_MYHELPER_H
//=============================================================================
