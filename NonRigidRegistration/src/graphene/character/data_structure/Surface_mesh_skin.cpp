//=============================================================================

#include <graphene/character/data_structure/Surface_mesh_skin.h>

//=============================================================================

namespace graphene {
namespace character {

//=============================================================================


Surface_mesh_skin::
Surface_mesh_skin() :
    Surface_mesh()
{
    visible_ = true;
    alpha_ = 1.0f;
    material_ = Vec4f(0.0f, 0.6f,0.0f,0.0f);
}


//-----------------------------------------------------------------------------

Surface_mesh_skin::Surface_mesh_skin(const Surface_mesh &mesh) :
    Surface_mesh(mesh)
{
    init_properties();
}

//-----------------------------------------------------------------------------

Surface_mesh_skin::
~Surface_mesh_skin()
{

}

//-----------------------------------------------------------------------------

void
Surface_mesh_skin::
init_properties()
{
    vertices_ = get_vertex_property<Point>("v:point");
    v_colors_ = get_vertex_property<Color>("v:color");
    v_normals_ = get_vertex_property<Normal>("v:normal");
    v_depends1_ = get_vertex_property<Vec4f>("v:skin_depend");
    v_weights1_ = get_vertex_property<Vec4f>("v:skin_weight");
    v_depends2_ = vertex_property<Vec4f>("v:skin_depend2", Vec4f(0.0f));
    v_weights2_ = vertex_property<Vec4f>("v:skin_weight2", Vec4f(0.0f));

    h_texcoords_ = get_halfedge_property<Texture_coordinate>("h:texcoord");
    h_normals_ = get_halfedge_property<Normal>("h:normal");

    Surface_mesh::Mesh_property<Vec3f>       ambient_color;
    Surface_mesh::Mesh_property<Vec3f>       diffuse_color;
    Surface_mesh::Mesh_property<Vec4f>       specular_color;
    Surface_mesh::Mesh_property<float>       alpha;

    ambient_color  = get_mesh_property<Vec3f>("m:ambient_color");
    diffuse_color  = get_mesh_property<Vec3f>("m:diffuse_color");
    specular_color = get_mesh_property<Vec4f>("m:specular_color");
    alpha          = get_mesh_property<float>("m:alpha");

    if (ambient_color)
    {
        const Vec3f &v = ambient_color[0];
        const float avg = (v[0]+v[1]+v[2]) / 3.0f;
        material_[0] = avg;
    }

    if (diffuse_color)
    {
        const Vec3f &v = diffuse_color[0];
        const float avg = (v[0]+v[1]+v[2]) / 3.0f;
        material_[1] = avg;
    }

    if (specular_color)
    {
        const Vec4f &v = specular_color[0];
        const float avg = (v[0]+v[1]+v[2]) / 3.0f;
        material_[2] = avg;
        material_[3] = v[3];
    }

    // HACK: remove this when we use the new template everywhere
    if ( (material_[0] + material_[1] + material_[2]) < 0.001f )
    {
        material_[1] = 0.6f;
    }

    if (alpha)
    {
        alpha_ = alpha[0];
    }
}

//-----------------------------------------------------------------------------

const geometry::Bounding_box
Surface_mesh_skin::
bbox()
{
    geometry::Bounding_box bbox;
    Vertex_iterator v_it;
    for (v_it=vertices_begin(); v_it != vertices_end(); ++v_it)
    {
        bbox += position(*v_it);
    }

    return bbox;
}


//=============================================================================
} // namespace character
} // namespace graphene
//=============================================================================
