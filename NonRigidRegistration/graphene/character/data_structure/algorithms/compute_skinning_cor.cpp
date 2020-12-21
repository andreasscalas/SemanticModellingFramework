#include <graphene/character/data_structure/algorithms/compute_skinning_cor.h>
#include <omp.h>

#include <float.h>
#include <vector>

namespace graphene
{
namespace character
{

double cor_similarity(const std::vector<double> &wp, const std::vector<double> &wv)
{
    const static double sigma = 0.05f;
    const static double divsig2 = 1.0f / (sigma*sigma);


    double sum = 0.0f;
    size_t j,k;
    double diff;
    double pjvk,pkvj;

    for (j=0; j < wp.size(); ++j)
    {
        if (wp[j] > 1e-8 && wv[j] > 1e-8)
        {
            for (k=0; k < wv.size(); ++k)
            {
                if (j!=k && wp[k] > 1e-8 && wv[k] > 1e-8)
                {
                    pjvk = wp[j]*wv[k];
                    pkvj = wp[k]*wv[j];
                    diff = pjvk - pkvj;
                    //if (pkvj > 0.000001f && pjvk > 0.000001f /*&& fabs(diff) < sigma*/)
                    {
                        sum += pjvk * pkvj * exp(- diff*diff * divsig2 );
                    }
                }

            }
        }
    }

    return sum;
}

inline void sum_vectors_div3(const std::vector<double>& in_v1, const std::vector<double>& in_v2, const std::vector<double>& in_v3, std::vector<double>& out_sum)
{
    out_sum.resize(in_v1.size());
    for (size_t i=0; i < in_v1.size(); ++i)
    {
        out_sum[i] = (in_v1[i] + in_v2[i] + in_v3[i]) * 1.0f/3.0f;
    }


}

//-----------------------------------------------------------------------------

void compute_skinning_corV(Surface_mesh *mesh, const std::string &prop_name, const int num_joints)
{
    //get props
    Surface_mesh::Vertex_property<Point> vpoint = mesh->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Vec4f> vweight = mesh->get_vertex_property<Vec4f>("v:skin_weight");
    Surface_mesh::Vertex_property<Vec4f> vdepend = mesh->get_vertex_property<Vec4f>("v:skin_depend");
    Surface_mesh::Vertex_property<Vec4f> vdepend2 = mesh->get_vertex_property<Vec4f>("v:skin_depend2");
    Surface_mesh::Vertex_property<Vec4f> vweight2 = mesh->get_vertex_property<Vec4f>("v:skin_weight2");

    if (! (vpoint && vweight))
    {
        std::cerr << "compute_skinning_cor: [ERROR] Cannot compute CoR. No skinning weights found." << std::endl;
        return;
    }
    Surface_mesh::Vertex v1,v2;


    std::vector< std::vector<double> > allweights(vpoint.vector().size(), std::vector<double>(num_joints,0.0f));

    for (size_t i=0; i < allweights.size(); ++i)
    {
        std::vector<double>& weights = allweights[i];

        v1 = Surface_mesh::Vertex((int) i);

        for (int j=0; j < 4; ++j)
        {
            const int d = (int) vdepend[v1][j];
            weights[d] = vweight[v1][j];
        }


        if (vdepend2 && vweight2)
        {
            for (int j=0; j < 4; ++j)
            {
                const int d = (int) vdepend2[v1][j];
                weights[d] = vweight2[v1][j];
            }
        }
    }

    Surface_mesh::Vertex_property<Vec3f> vcor = mesh->vertex_property<Vec3f>(prop_name);

    double sim, n;

    const int vsize = (int)vpoint.vector().size();
#pragma omp parallel for private(v1,v2,sim,n)
    for (int vi1 = 0; vi1 < vsize; ++vi1)
    {
        v1 = Surface_mesh::Vertex((int)vi1);

        Vec3f& cor = vcor[v1];

        cor = Vec3f(0.0f);
        n = 0.0f;

        for (size_t vi2 = 0; vi2 < vpoint.vector().size(); ++vi2)
        {
            v2 = Surface_mesh::Vertex((int)vi2);

            sim = cor_similarity(allweights[vi1], allweights[vi2]);
            cor += sim * vpoint[v2];
            n += sim;
        }

        if(fabs(n) <= FLT_EPSILON )
            cor = Vec3f(0.0,0.0,0.0);
        else
            cor *= 1.0 / n;
    }
}


//-----------------------------------------------------------------------------


void compute_skinning_corT(Surface_mesh *mesh, const std::string &prop_name, const int num_joints)
{
    //get props
    Surface_mesh::Vertex_property<Point> vpoint = mesh->get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Vec4f> vweight = mesh->get_vertex_property<Vec4f>("v:skin_weight");
    Surface_mesh::Vertex_property<Vec4f> vdepend = mesh->get_vertex_property<Vec4f>("v:skin_depend");
    Surface_mesh::Vertex_property<Vec4f> vdepend2 = mesh->get_vertex_property<Vec4f>("v:skin_depend2");
    Surface_mesh::Vertex_property<Vec4f> vweight2 = mesh->get_vertex_property<Vec4f>("v:skin_weight2");

    if (! (vpoint && vweight))
    {
        std::cerr << "compute_skinning_cor: [ERROR] Cannot compute CoR. No skinning weights found." << std::endl;
        return;
    }

    Surface_mesh::Vertex_property<Vec3f> vcor = mesh->vertex_property<Vec3f>(prop_name);

    Surface_mesh::Vertex vi,va,vb,vc;


    std::vector< std::vector<double> > allweights(vpoint.vector().size(), std::vector<double>(num_joints,0.0));

    for (size_t i=0; i < allweights.size(); ++i)
    {
        std::vector<double>& weights = allweights[i];

        vi = Surface_mesh::Vertex((int) i);

        for (int j=0; j < 4; ++j)
        {
            const int d = (int) vdepend[vi][j];
            weights[d] = vweight[vi][j];
        }

        if (vdepend2 && vweight2)
        {
            for (int j=0; j < 4; ++j)
            {
                const int d = (int) vdepend2[vi][j];
                weights[d] = vweight2[vi][j];
            }
        }
    }

    double sim, n, area;

    int c=0;

    const int vsize = (int)vpoint.vector().size();
#pragma omp parallel for private(vi,va,vb,vc,sim,n,area)
    for (int vi1 = 0; vi1 < vsize; ++vi1)
    {
        vi = Surface_mesh::Vertex((int)vi1);

        n = 0.0;
        vcor[vi] = Vec3f(0.0);
        c = 0;

        Surface_mesh::Face_iterator fit;
        Surface_mesh::Vertex_around_face_circulator vfc;
        std::vector<double> w_sum;

        for (fit = mesh->faces_begin(); fit != mesh->faces_end(); ++fit)
        {
            vfc = mesh->vertices(*fit);

            va = *vfc;
            ++vfc;
            vb = *vfc;
            ++vfc;
            vc = *vfc;

            const Point& pa = vpoint[va];
            const Point& pb = vpoint[vb];
            const Point& pc = vpoint[vc];

            area = 0.5f*norm(cross(pb - pa, pc - pa));

            sum_vectors_div3(allweights[va.idx()], allweights[vb.idx()], allweights[vc.idx()], w_sum);

            //sim = cor_similarityT(vweight[vi], vdepend[vi], wa, wb, wc, da, db, dc) * area;
            sim = cor_similarity(allweights[vi.idx()], w_sum) *area;
            n += sim;
            vcor[vi] += sim * ( (pa+pb+pc) / 3.0 );
        }

        if(fabs(n) <= FLT_EPSILON)
            vcor[vi] = Vec3f(0.0,0.0,0.0);
        else
            vcor[vi] *= 1.0 / n;
    }
}

}
}
