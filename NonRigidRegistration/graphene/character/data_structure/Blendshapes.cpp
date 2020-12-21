//=============================================================================

#include <graphene/character/data_structure/Blendshapes.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/crease_normals.h>

//=============================================================================

using namespace graphene::surface_mesh;

namespace graphene {
namespace character {

//=============================================================================


Blendshapes::Blendshapes() :
    base_(NULL)
{
}

Blendshapes::~Blendshapes()
{
    clear();
}

void Blendshapes::clear()
{
    size_t i;

    for (i=0; i < targets_.size(); ++i)
    {
        delete targets_[i];
    }
    targets_.clear();
    base_ = NULL;

    moving_indices_.clear();
}

void Blendshapes::clear_blended()
{
    if (base_ != NULL)
    {
        base_->remove_vertex_property(blended_vertices_);
        base_->remove_vertex_property(blended_v_normals_);
        base_->remove_halfedge_property(blended_h_normals_);
    }
}


void Blendshapes::add_target(Target_shape *target)
{
    target->init_properties();

    targets_.push_back(target);

    if (target->n_faces() == 0)
        return;


    bool need_repair = false;

    Surface_mesh::Face_iterator fit_base=base_->faces_begin(),fit_target=target->faces_begin();
    Surface_mesh::Face ft,fb;
    Surface_mesh::Vertex_around_face_circulator vfc_base,vfc_end_base,vfc_target,vfc_end_target;
    int c,c_all;
    while (true)
    {
        ft = *fit_target;
        fb = *fit_base;


        c_all = c = 0;
        vfc_base = vfc_end_base = base_->vertices(fb);
        vfc_target = vfc_end_target = target->vertices(ft);
        do
        {
            if (*vfc_base == *vfc_target)
                ++c;

            ++c_all;

            if (++vfc_base != vfc_end_base || ++vfc_target != vfc_end_target)
                break;
        }
        while(true);

        if (c_all != c)
        {
            need_repair = true;
            break;
        }

        ++fit_base;
        ++fit_target;
        if (fit_base == base_->faces_end() || fit_target == target->faces_end())
            break;
    }

    if (need_repair)
    {
        std::cerr << "Blendshapes::add_target: [WARNING] Target mesh has different topology. Using topology of base mesh. Normals will be recalculated!" << std::endl;

        //copy mesh connectivity from base mesh to target
        Surface_mesh::Vertex_property<Surface_mesh::Vertex_connectivity> vc_base, vc_target;
        Surface_mesh::Halfedge_property<Surface_mesh::Halfedge_connectivity> hc_base, hc_target;
        Surface_mesh::Face_property<Surface_mesh::Face_connectivity> fc_base, fc_target;

        vc_base = base_->get_vertex_property<Surface_mesh::Vertex_connectivity>("v:connectivity");
        vc_target = target->get_vertex_property<Surface_mesh::Vertex_connectivity>("v:connectivity");

        hc_base = base_->get_halfedge_property<Surface_mesh::Halfedge_connectivity>("h:connectivity");
        hc_target = target->get_halfedge_property<Surface_mesh::Halfedge_connectivity>("h:connectivity");

        fc_base = base_->get_face_property<Surface_mesh::Face_connectivity>("f:connectivity");
        fc_target = target->get_face_property<Surface_mesh::Face_connectivity>("f:connectivity");

        vc_target.vector() = vc_base.vector();
        hc_target.vector() = hc_base.vector();
        fc_target.vector() = fc_base.vector();

        crease_normals(target, 0.0f, "h:normal");
    }
}


void Blendshapes::set_active(const std::vector<float> &weights, const std::vector<unsigned int> &indices)
{
    active_weights_ = weights;
    active_indices_ = indices;
}

void Blendshapes::clear_active()
{
    active_indices_.clear();
    active_weights_.clear();
}

bool Blendshapes::apply()
{
    if (base_ == NULL || active_weights_.size() != active_indices_.size())
        return false;


    if (active_indices_.empty())
        return false;


    if (! blended_vertices_)
    {
        blended_vertices_ = base_->vertex_property<Point>("v:blended_point");

        if (base_->v_normals_)
        {
            blended_v_normals_ = base_->vertex_property<Normal>("v:blended_normal");
            for (auto v : base_->vertices())
            {
                blended_v_normals_[v] = base_->v_normals_[v];
            }
        }
        else if (base_->h_normals_)
        {
            blended_h_normals_ = base_->halfedge_property<Normal>("h:blended_normal");
        }

        for (auto v : base_->vertices())
        {
            blended_vertices_[v]  = base_->vertices_[v];
        }
    }

    size_t i;
    surface_mesh::Surface_mesh::Vertex_iterator vit;
    Target_shape* target;
#pragma omp parallel for private(target)
    for (int cnt = 0; cnt < (int) moving_indices_.size(); ++cnt)
    {
        graphene::surface_mesh::Surface_mesh::Vertex v(moving_indices_[cnt]);

        blended_vertices_[v]  = Point(0.0f);
        if (blended_v_normals_)
        {
            blended_v_normals_[v] = base_->v_normals_[v];
        }


        for (i=0; i < active_indices_.size(); ++i)
        {
            target = targets_[active_indices_[i]];

            blended_vertices_[v] += active_weights_[i] * (target->vertices_[v] - base_->vertices_[v]);
            if (blended_v_normals_ && target->v_normals_)
            {
                blended_v_normals_[v] += active_weights_[i] * target->v_normals_[v];
            }
            /*
            else if (blended_h_normals_ && target->h_normals_)
            {
            }
            */
        }

        if (blended_v_normals_)
        {
            blended_v_normals_[v] = normalize(blended_v_normals_[v]);
        }
        blended_vertices_[v]  = base_->vertices_[v] + blended_vertices_[v];
    }

    return true;
}


void
Blendshapes::
update_moving_indizes()
{
    moving_indices_.clear();

    for (auto v : base_->vertices())
    {
        for (unsigned int i = 0; i < targets_.size(); ++i)
        {
            const double diff = distance(targets_[i]->vertices_[v], base_->vertices_[v]);
            if (diff > 1e-7)
            {
                moving_indices_.push_back(v.idx());
                break;
            }
        }
    }
}


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
