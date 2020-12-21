//=============================================================================
#ifndef GRAPHENE_MEANVALUE_COORDS_H
#define GRAPHENE_MEANVALUE_COORDS_H
//=============================================================================

//== INCLUDES ===================================================================

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>


//=============================================================================
namespace graphene
{
namespace surface_mesh
{
namespace mvc
{

static void compute_mean_value_coords(Surface_mesh* mesh, const std::vector<Point>& points, const std::string &prop_name)
{
    double w_sum,w_i,r,beta_jk,beta_ij,beta_ki;
    Vec3d v_i, e_i, v_j, e_j, v_k, e_k, n_jk,n_ij,n_ki;
    size_t i,j;
    surface_mesh::Surface_mesh::Mesh_property< std::vector< std::vector < float > > > mvc_jpos_meshp = mesh->mesh_property<std::vector< std::vector <float> > >(prop_name);
    surface_mesh::Surface_mesh::Vertex_property<Point> vpoint = mesh->get_vertex_property<Point>("v:point");
    surface_mesh::Surface_mesh::Vertex_iterator v_it;
    surface_mesh::Surface_mesh::Vertex_around_vertex_circulator vvc;
    surface_mesh::Surface_mesh::Vertex_around_vertex_circulator vvc_end;


    std::vector< std::vector <float> > &mvc_jpos = mvc_jpos_meshp[0];
    mvc_jpos.clear();
    mvc_jpos.resize(points.size());


    for (i=0; i < points.size(); ++i)
    {
        std::vector<float> &mvc_weights = mvc_jpos[i];

        mvc_weights.resize(mesh->n_vertices());

        w_sum = 0.0;

        for (v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            v_i = vpoint[*v_it] - points[i];

            w_i = 0.0;
            r   = 1.0 / norm(v_i);
            e_i = v_i.normalize();


            //loop local neighborhood
            vvc = mesh->vertices(*v_it);
            if (mesh->is_boundary(*vvc))
                continue;
            vvc_end = vvc;
            do
            {
               v_j = vpoint[*vvc] - points[i];
               e_j = v_j.normalize();
               if (mesh->is_boundary(*vvc))
               {
                   w_i = 0.0;
                   break;
               }

               ++vvc; // !! vvc increased from here on !!
               if (mesh->is_boundary(*vvc))
               {
                   w_i = 0.0;
                   break;
               }
               v_k = vpoint[*vvc] - points[i];
               e_k = v_k.normalize();

               beta_jk = std::abs(acos(dot(e_j, e_k)));
               beta_ij = std::abs(acos(dot(e_i, e_j)));
               beta_ki = std::abs(acos(dot(e_k, e_i)));

               n_jk = cross(e_j, e_k).normalize();
               n_ij = cross(e_i, e_j).normalize();
               n_ki = cross(e_k, e_i).normalize();

               double w = beta_jk + beta_ij*dot(n_ij, n_jk) + beta_ki*dot(n_ki, n_jk);

               if (!(w == w))
               {
                   std::cerr << "Character::compute_mean_value_coordinates: [ERROR] Weight is nan!" << std::endl;
               }

               w *= 1.0 / (2.0*dot(e_i, n_jk));
               w_i += w;
            } while (vvc != vvc_end);



            w_i *= r;
            //if (w_i < 0){
            //   std::cerr << "w_i is smaller 0:" << w_i << std::endl;
            //}
            w_sum += w_i;
            mvc_weights[(*v_it).idx()] = w_i;
        }

        // normalization of all weights
        for (j = 0; j < mvc_weights.size(); ++j)
        {
           mvc_weights[j] /= w_sum;
        }


        Point p_comp(0.0);
        for (v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            p_comp += mvc_weights[(*v_it).idx()] * vpoint[*v_it];
        }
        float d = distance(p_comp, points[i]);
        if (d > 0.001)
        {
           std::cerr << "MVC points difference: Distance = " << d << " | real position "<<p_comp<<" -> calc. position "<<points[i]<< std::endl;
        }
    }
}

static
bool
get_points(const Surface_mesh* mesh, std::vector<Point>& points, const std::string& prop_name)
{
    Surface_mesh::Mesh_property<std::vector< std::vector< float > > > mvc_prop =
            mesh->get_mesh_property<std::vector< std::vector< float > > >(prop_name);

    surface_mesh::Surface_mesh::Vertex_property<Point> vpoint =
            mesh->get_vertex_property<Point>("v:point");

    if (!mvc_prop || ! vpoint)
    {
        return false;
    }

    size_t i;
    surface_mesh::Surface_mesh::Vertex_iterator v_it;
    Point p;

    std::vector< std::vector <float> > &mvc_jpos = mvc_prop[0];

    points.clear();
    points.resize(mvc_jpos.size());

    for (i=0; i < points.size(); ++i)
    {
        std::vector<float> &mvc_weights = mvc_jpos[i];

        p=Vec3f(0.0f);
        for (v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            p += mvc_weights[(*v_it).idx()] * vpoint[*v_it];
        }

        points[i] = p;
    }

    return true;
}
}
}
}


//=============================================================================
#endif // GRAPHENE_MEANVALUE_COORDS_H
//=============================================================================

