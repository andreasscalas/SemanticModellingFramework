//=============================================================================

#include <graphene/character/scene_graph/Character_node.h>
#include <graphene/surface_mesh/algorithms/surface_mesh_tools/crease_normals.h>
#include <sstream>
#include <set>

//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


Character_node::
Character_node(const std::string& name) :
    Surface_mesh_node(name),
    active_mesh_color_(0.2f,0.75f,0.2f)
{
    clear_draw_modes();

    add_draw_mode("Solid");
    add_draw_mode("Solid Edge");
    add_draw_mode("Vertex_color");
    add_draw_mode("Wireframe");
    add_draw_mode("Textured (Shaded)");
    add_draw_mode("Textured");
    add_draw_mode("Textured Edge");
    add_draw_mode("Skeleton");
    add_draw_mode("Skeleton only");
    add_draw_mode("Points");
    add_draw_mode("Unskinned");


    set_draw_mode("Solid");

    // init more settings for draw modes
    clear_draw_more_settings();

    // material parameters
    front_color_  = Vec3f(0.4, 0.425, 0.475);
    back_color_   = Vec3f(0.5, 0.3, 0.3);
    wire_color_   = Vec3f(0,0,0);
    material_     = Vec4f(0.0, 0.6, 0.0, 64.0);
}


//-----------------------------------------------------------------------------


Character_node::
~Character_node()
{
    size_t i;

    for (i=0; i < skin_gl_data_.size(); ++i)
    {
        delete skin_gl_data_[i];
    }
}


//-----------------------------------------------------------------------------


surface_mesh::Surface_mesh&
Character_node::
mesh()
{
    return *character_.skins()[character_.get_selected_skin_idx()];
}


//-----------------------------------------------------------------------------

void Character_node::set_texture(gl::Texture *texture, gl::Texture_type type)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];

    switch (type)
    {
    case gl::TT_COLOR:
        sgd->texture_ = texture;
        break;
    case gl::TT_NORMAL:
        sgd->texture_nm_ = texture;
        break;
    case gl::TT_SPECULAR:
        sgd->texture_spec_ = texture;
        break;
    default:
        break;
    }

    update_skin_texture(sgd);
}

//-----------------------------------------------------------------------------

gl::Texture* Character_node::get_texture(gl::Texture_type type)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];

    switch (type)
    {
    case gl::TT_COLOR:
        return sgd->texture_;
        break;
    case gl::TT_NORMAL:
        return sgd->texture_nm_;
        break;
    case gl::TT_SPECULAR:
        return sgd->texture_spec_;
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------


void
Character_node::
apply_transform(const Mat4f &transform)
{
    size_t i;
    character::Surface_mesh_skin* skin;
    Surface_mesh::Vertex_iterator v_it;
    Surface_mesh::Halfedge_iterator h_it;
    Mat3f normal_mat = inverse(transpose(Mat3f(transform)));
    for (i=0; i < character_.skins().size(); ++i)
    {
        skin = character_.skins()[i];

        for (v_it = skin->vertices_begin(); v_it != skin->vertices_end(); ++v_it)
        {
            Point &p = skin->vertices_[*v_it];
            p = affine_transform(transform, p);

            if (skin->v_normals_)
            {
                Normal &n = skin->v_normals_[*v_it];
                n = normal_mat * n;
            }
        }
        update_skin_points(skin_gl_data_[i]);

        if (skin->h_normals_)
        {
            for (h_it = skin->halfedges_begin(); h_it != skin->halfedges_end(); ++h_it)
            {
                Normal &n = skin->h_normals_[*h_it];
                n = normal_mat * n;
            }
        }

        if (skin->h_normals_ || skin->v_normals_)
        {
            update_skin_normals(skin_gl_data_[i]);
        }
    }
    for (i=0; i < character_.blendshapes().size(); ++i)
    {
        graphene::character::Blendshapes* blendshape = character_.blendshapes()[i];

        std::vector<graphene::character::Blendshapes::Target_shape*>& targets = blendshape->targets();
        for (unsigned int j = 0; j < targets.size(); ++j)
        {
            graphene::character::Blendshapes::Target_shape* target = targets[j];
            for (v_it = target->vertices_begin(); v_it != target->vertices_end(); ++v_it)
            {
                Point &p = target->vertices_[*v_it];
                p = affine_transform(transform, p);

                if (target->v_normals_)
                {
                    Normal &n = target->v_normals_[*v_it];
                    n = normal_mat * n;
                }
            }
            update_skin_points(blendshapes_skin_gl_data_[i]);

            if (target->h_normals_)
            {
                for (h_it = target->halfedges_begin(); h_it != target->halfedges_end(); ++h_it)
                {
                    Normal &n = target->h_normals_[*h_it];
                    n = normal_mat * n;
                }
            }

            if (target->h_normals_ || target->v_normals_)
            {
                update_skin_normals(blendshapes_skin_gl_data_[i]);
            }
        }
    }

    bbox_ = character_.bbox();
}


//-----------------------------------------------------------------------------


bool
Character_node::
load(const std::string &filename)
{
    if ( !character_.read(filename.c_str()) )
    {
        return false;
    }

    fileinfo_ = filename;

    character_.skeleton().init();

    bbox_ = character_.bbox();

    return true;
}


//-----------------------------------------------------------------------------


bool
Character_node::
save(const std::string &filename)
{
    std::string ext;
    ext = filename.substr(filename.rfind('.') + 1);

    if (ext == "bim" || ext == "fbx")
    {
        std::string current_basename;
        current_basename = this->fileinfo().substr(this->fileinfo().rfind('/') + 1);
        current_basename = current_basename.substr(0, current_basename.rfind('.'));

        std::string target_basename;
        target_basename = filename.substr(filename.rfind('/') + 1);
        target_basename = target_basename.substr(0, target_basename.rfind('.'));

        size_t i;
        character::Surface_mesh_skin* mesh;
        const std::vector<character::Surface_mesh_skin*> &skins = character_.skins();

        surface_mesh::Surface_mesh::Mesh_property<Vec3f> m_ambient_color;
        surface_mesh::Surface_mesh::Mesh_property<Vec3f> m_diffuse_color;
        surface_mesh::Surface_mesh::Mesh_property<Vec4f> m_specular_color;
        surface_mesh::Surface_mesh::Mesh_property<float> m_alpha;

        surface_mesh::Surface_mesh::Mesh_property<std::string> texname;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname_nm;
        surface_mesh::Surface_mesh::Mesh_property<std::string> texname_spec;

        //set material for all skins as mesh property, if not already present
        for (i=0; i < skins.size(); ++i)
        {
            mesh = skins[i];

            {
                m_ambient_color  = mesh->mesh_property<Vec3f>("m:ambient_color");
                m_diffuse_color  = mesh->mesh_property<Vec3f>("m:diffuse_color");
                m_specular_color = mesh->mesh_property<Vec4f>("m:specular_color");
                m_alpha          = mesh->mesh_property<float>("m:alpha");

                m_ambient_color[0]  = Vec3f(mesh->material_[0]);
                m_diffuse_color[0]  = Vec3f(mesh->material_[1]);
                m_specular_color[0] = Vec4f(mesh->material_[2], mesh->material_[2], mesh->material_[2], mesh->material_[3]);
                m_alpha[0]          = mesh->alpha_;
            }

            texname = mesh->get_mesh_property<std::string>("m:texturename");
            if (texname)
            {
                std::string& texfilename = texname[0];

                auto tmp_texname = mesh->add_mesh_property<std::string>("m:tmp_texturename");
                tmp_texname[0] = texfilename;

                size_t start_pos = texfilename.rfind(current_basename);
                if(start_pos != std::string::npos)
                {
                    texfilename.replace(start_pos, current_basename.length(), target_basename);
                }
            }

            texname_nm = mesh->get_mesh_property<std::string>("m:texturename_nm");
            if (texname_nm)
            {
                std::string& texfilename = texname_nm[0];

                auto tmp_texname = mesh->add_mesh_property<std::string>("m:tmp_texturename_nm");
                tmp_texname[0] = texfilename;

                size_t start_pos = texfilename.rfind(current_basename);
                if(start_pos != std::string::npos)
                {
                    texfilename.replace(start_pos, current_basename.length(), target_basename);
                }
            }

            texname_spec = mesh->get_mesh_property<std::string>("m:texturename_spec");
            if (texname_spec)
            {
                std::string& texfilename = texname_spec[0];

                auto tmp_texname = mesh->add_mesh_property<std::string>("m:tmp_texturename_spec");
                tmp_texname[0] = texfilename;

                size_t start_pos = texfilename.rfind(current_basename);
                if(start_pos != std::string::npos)
                {
                    texfilename.replace(start_pos, current_basename.length(), target_basename);
                }
            }
        }

        bool write_ok = false;
        if (ext == "bim")
        {
            write_ok = character_.write_bim(filename.c_str());
        }
        else if (ext == "fbx")
        {
            write_ok = character_.write_fbx(filename.c_str());
        }

        // restore previous texture names
        for (i=0; i < skins.size(); ++i)
        {
            mesh = skins[i];

            texname = mesh->get_mesh_property<std::string>("m:texturename");
            if (texname)
            {
                std::string& texfilename = texname[0];

                auto tmp_texname = mesh->get_mesh_property<std::string>("m:tmp_texturename");
                texfilename = tmp_texname[0];

                mesh->remove_mesh_property(tmp_texname);
            }

            texname_nm = mesh->get_mesh_property<std::string>("m:texturename_nm");
            if (texname_nm)
            {
                std::string& texfilename = texname_nm[0];

                auto tmp_texname = mesh->get_mesh_property<std::string>("m:tmp_texturename_nm");
                texfilename = tmp_texname[0];

                mesh->remove_mesh_property(tmp_texname);
            }

            texname_spec = mesh->get_mesh_property<std::string>("m:texturename_spec");
            if (texname_spec)
            {
                std::string& texfilename = texname_spec[0];

                auto tmp_texname = mesh->get_mesh_property<std::string>("m:tmp_texturename_spec");
                texfilename = tmp_texname[0];

                mesh->remove_mesh_property(tmp_texname);
            }
        }

        //save texture with correct filename in same directory
        if (write_ok)
        {
            gl::Texture* colortex = get_texture(gl::TT_COLOR);
            if (colortex != nullptr)
            {
                std::string texfilename = colortex->name_;

                //extract filename
                size_t p = texfilename.rfind('/');
                if (p != std::string::npos)
                    texfilename = texfilename.substr(p+1);

                //replace original basename with new name of model file (bim/fbx)
                size_t start_pos = texfilename.rfind(current_basename);
                if(start_pos != std::string::npos)
                {
                    texfilename.replace(start_pos, current_basename.length(), target_basename);
                }

                //create full filename
                std::string full_fname = filename;
                p = full_fname.rfind('/');
                if (p != std::string::npos)
                    full_fname = full_fname.substr(0, p+1);

                full_fname.append(texfilename);


                if (!colortex->write(full_fname))
                {
                    std::cout << "Character_node::save: [ERROR] Color texture not saved!" << std::endl;
                }
            }
        }

        return write_ok;
    }
    else
    {
        const graphene::character::Surface_mesh_skin* selected_skin = character_.get_selected_skin();
        const std::vector<graphene::character::Blendshapes*>& blendshapes = character_.blendshapes();
        bool blendshape_found = false;
        unsigned int bs_idx = 0;
        for (unsigned int i = 0; i < blendshapes.size(); ++i)
        {
            if (selected_skin == blendshapes[i]->base())
            {
                bs_idx = i;
                blendshape_found = true;
                break;
            }
        }
        if (blendshape_found)
        {
            auto points           = blendshapes[bs_idx]->base()->get_vertex_property<Point>("v:point");
            auto blended_vertices = blendshapes[bs_idx]->base()->get_vertex_property<Point>("v:blended_point");
            bool write_ok = false;
            if (points && blended_vertices)
            {
                for (auto v : blendshapes[bs_idx]->base()->vertices())
                {
                    std::swap(points[v], blended_vertices[v]);
                }
                write_ok = blendshapes[bs_idx]->base()->write(filename);
                for (auto v : blendshapes[bs_idx]->base()->vertices())
                {
                    std::swap(points[v], blended_vertices[v]);
                }
                return write_ok;
            }
            else
            {
                return selected_skin->write(filename);
            }
        }
        else
        {
            return selected_skin->write(filename);
        }
    }
}


//-----------------------------------------------------------------------------


std::string
Character_node::
info() const
{
    character::Surface_mesh_skin* skin;
    unsigned int n_vertices=0, n_faces=0;
    for (size_t i=0; i < character_.skins().size(); ++i)
    {
        skin = character_.skins()[i];
        n_vertices += skin->n_vertices();
        n_faces += skin->n_faces();
    }

    std::ostringstream s;
    s << character_.skins().size()
      << " skins and skeleton with "
      << character_.skeleton().joints_.size()
      << " joints. Overall "
      << n_vertices
      << " vertices and "
      << n_faces << " faces.";
    return s.str();
}


//-----------------------------------------------------------------------------


void
Character_node::
clear_selections()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    auto selected = mesh->get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh->vertices())
        {
            selected[v] = false;
        }
        update_skin_selection(sgd);

        mesh->remove_vertex_property(selected);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
invert_selections()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    auto selected = mesh->get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh->vertices())
        {
            selected[v] = !selected[v];
        }
        update_skin_selection(sgd);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
select_all()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;


    auto selected = mesh->vertex_property<bool>("v:selected");

    for (auto v: mesh->vertices())
    {
        selected[v] = true;
    }
    update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
select_isolated()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;
    Surface_mesh::Vertex_property<bool> selected;
    for (auto v: mesh->vertices())
    {
        if (mesh->is_isolated(v))
        {
            if (!selected)
                selected = mesh->vertex_property<bool>("v:selected");

            selected[v] = true;
        }
    }
    if (selected)
        update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
delete_selected()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;
    Surface_mesh::Vertex_property<bool> selected = mesh->get_vertex_property<bool>("v:selected");

    std::vector<graphene::character::Blendshapes*>& blendshapes = character_.blendshapes();
    graphene::character::Blendshapes* blendshape = 0;
    for (unsigned int k = 0; k < blendshapes.size(); ++k)
    {
        if (mesh == blendshapes[k]->base())
        {
            blendshape = blendshapes[k];
            break;
        }
    }

    if (selected)
    {
        for (auto v: mesh->vertices())
        {
            if (selected[v])
            {
                selected[v] = false;
                mesh->delete_vertex(v);

                if (blendshape)
                {
                    for (unsigned int l = 0; l < blendshape->targets().size(); ++l)
                    {
                        blendshape->targets()[l]->delete_vertex(v);
                    }
                }
            }
        }
        mesh->garbage_collection();
        update_mesh();
        if (blendshape)
        {
            for (unsigned int l = 0; l < blendshape->targets().size(); ++l)
            {
                blendshape->targets()[l]->garbage_collection();
            }
            blendshape->update_moving_indizes();
            update_blendshapes();
        }
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
grow_selection()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    std::vector<Surface_mesh::Vertex> vertices;
    auto selected = mesh->get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v : mesh->vertices())
        {
            if (selected[v])
            {
                for (auto vv: mesh->vertices(v))
                {
                    vertices.push_back(vv);
                }
            }
        }

        for (auto v : vertices)
            selected[v] = true;

        update_skin_selection(sgd);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
set_selection(const std::vector<size_t> &indices)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    auto selected = mesh->vertex_property<bool>("v:selected");

    for (size_t i=0; i < selected.vector().size(); i++)
    {
        selected.vector()[i] = false;
    }

    for (size_t i(0); i < indices.size(); i++)
    {
        selected[Surface_mesh::Vertex((int)indices[i])] = true;
    }
    update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
add_selection(const std::vector<size_t> &indices)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    auto selected = mesh->vertex_property<bool>("v:selected");


    for (size_t i(0); i < indices.size(); i++)
    {
        selected[Surface_mesh::Vertex(indices[i])] = true;
    }
    update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
get_selection(std::vector<size_t>& indices)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    indices.clear();
    auto selected = mesh->get_vertex_property<bool>("v:selected");

    if (selected)
    {
        for (auto v: mesh->vertices())
        {
            if (selected[v])
            {
                indices.push_back(v.idx());
            }
        }
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
clear_selection(const std::vector<size_t>& indices)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    auto selected = mesh->get_vertex_property<bool>("v:selected");
    if (selected)
    {
        for (auto i : indices)
        {
            selected[Surface_mesh::Vertex((int)i)] = false;
        }
        update_skin_selection(sgd);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
select_point(Point p)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    Surface_mesh::Vertex v_min;
    Scalar  d, d_min(FLT_MAX);

    auto points   = mesh->vertex_property<Point>("v:point");
    auto selected = mesh->vertex_property<bool>("v:selected");
    auto blended_vertices = mesh->get_vertex_property<Point>("v:blended_point");

    for (auto v: mesh->vertices())
    {
        if (blended_vertices)
        {
            d = distance(blended_vertices[v], (Point)p);
        }
        else
        {
            d = distance(points[v], (Point)p);
        }
        if (d < d_min)
        {
            v_min = v;
            d_min = d;
        }
    }
    selected[v_min] = true;
    update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
select_landmark(const Point& p)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    Surface_mesh::Vertex v_min;
    Scalar  d, d_min(FLT_MAX);

    auto points    = mesh->vertex_property<Point>("v:point");
    auto selected  = mesh->vertex_property<bool>("v:selected");
    auto blended_vertices = mesh->get_vertex_property<Point>("v:blended_point");
    auto landmarks = mesh->mesh_property< std::vector<Surface_mesh::Vertex> >("m:landmarks");
    std::vector<Surface_mesh::Vertex> &lm = landmarks[0];

    auto landmarks_lw = mesh->get_vertex_property< double >("v:landmark_weight");

    for (auto v: mesh->vertices())
    {
        if (blended_vertices)
        {
            d = distance(blended_vertices[v], (Point)p);
        }
        else
        {
            d = distance(points[v], (Point)p);
        }
        if (d < d_min)
        {
            v_min = v;
            d_min = d;
        }
    }

    lm.push_back(v_min);
    if (landmarks_lw)
    {
        landmarks_lw[v_min] = 1.0;
    }
    selected[v_min] = true;
    update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


void Character_node::
clear_landmarks()
{
    auto landmarks    = mesh().get_mesh_property< std::vector<Surface_mesh::Vertex> > ("m:landmarks");
    auto landmarks_lw = mesh().get_vertex_property< double > ("v:landmark_weight");
    if (landmarks)
    {
        mesh().remove_mesh_property(landmarks);
    }
    if (landmarks_lw)
    {
        mesh().remove_vertex_property(landmarks_lw);
    }
}


//-----------------------------------------------------------------------------


void Character_node::
highlight_landmark(int idx)
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    Surface_mesh::Mesh_property< std::vector< Surface_mesh::Vertex > > landmarks = mesh->get_mesh_property< std::vector< Surface_mesh::Vertex > >("m:landmarks");

    if (! landmarks)
    {
        sgd->selection_highlight_idx_ = -1;
        return;
    }

    std::vector<Surface_mesh::Vertex> &lm = landmarks[0];

    Surface_mesh::Vertex v = lm[idx];
    Surface_mesh::Vertex_property<size_t> vindices = mesh->get_vertex_property<size_t>("v:index");
    if (vindices && idx != -1)
    {
        sgd->selection_highlight_idx_ = (int) vindices[v];
    }
    else
    {
        sgd->selection_highlight_idx_ = -1;
    }

    update_skin_selection(sgd);
}


//-----------------------------------------------------------------------------


int Character_node::
get_num_landmarks()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    Surface_mesh::Mesh_property< std::vector< Surface_mesh::Vertex > > landmarks = mesh->get_mesh_property< std::vector< Surface_mesh::Vertex > >("m:landmarks");
    if (landmarks)
    {
        std::vector<Surface_mesh::Vertex> &lm = landmarks[0];

        if (lm.empty())
        {
            sgd->selection_highlight_idx_ = -1;
            return 0;
        }
        else
        {
            return (int) lm.size();
        }
    }
    else
    {
        sgd->selection_highlight_idx_ = -1;
        return 0;
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
init(gl::GL_state *gls)
{

    size_t i;
    character::Surface_mesh_skin* skin;
    Skin_gl_data* sgd;


    for (i=0; i < character_.skins().size(); ++i)
    {
        skin = character_.skins()[i];

        sgd = new Skin_gl_data(skin);

        for (unsigned int k = 0; k < character_.blendshapes().size(); ++k)
        {
            if (skin == character_.blendshapes()[k]->base())
            {
                blendshapes_skin_gl_data_.push_back(sgd);
                sgd->blendshapes_ = character_.blendshapes()[k];
            }
        }

        init_skin_gl_data(sgd, gls);

        skin_gl_data_.push_back(sgd);
    }


    set_selected_skin(0);

    if (character_.skeleton().joints_.empty())
    {
        remove_draw_mode("Skeleton");
        remove_draw_mode("Skeleton only");
    }

    init_skeleton_gl_data();
    update_skeleton();
}


//-----------------------------------------------------------------------------


void
Character_node::
add_skin(graphene::character::Surface_mesh_skin* new_skin, gl::GL_state *gls)
{
    Skin_gl_data* sgd = new Skin_gl_data(new_skin);

    init_skin_gl_data(sgd, gls);

    skin_gl_data_.push_back(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_mesh()
{
    recompute_normals(skin_gl_data_[character_.get_selected_skin_idx()]);

    update_skin_points(skin_gl_data_[character_.get_selected_skin_idx()]);
    update_skin_normals(skin_gl_data_[character_.get_selected_skin_idx()]);
    update_skin_texcoords(skin_gl_data_[character_.get_selected_skin_idx()]);
    update_skin_weight_and_depends(skin_gl_data_[character_.get_selected_skin_idx()]);
    update_skin_selection(skin_gl_data_[character_.get_selected_skin_idx()]);

    update_skeleton();

    bbox_ = character_.bbox();
}


//-----------------------------------------------------------------------------


void
Character_node::
update_meshes()
{
    for (unsigned int i = 0; i < skin_gl_data_.size(); ++i)
    {
        update_skin_points(skin_gl_data_[i]);
        update_skin_normals(skin_gl_data_[i]);
        update_skin_texcoords(skin_gl_data_[i]);
        update_skin_weight_and_depends(skin_gl_data_[i]);
        update_skin_selection(skin_gl_data_[i]);
    }

    update_skeleton();

    bbox_ = character_.bbox();
}


//-----------------------------------------------------------------------------


void
Character_node::
update_colors()
{
    for (unsigned int i = 0; i < skin_gl_data_.size(); ++i)
    {
        update_skin_colors(skin_gl_data_[i]);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
update_blendshapes()
{
    for (unsigned int i = 0; i < blendshapes_skin_gl_data_.size(); ++i)
    {
        update_skin_points(blendshapes_skin_gl_data_[i]);
        update_skin_normals(blendshapes_skin_gl_data_[i]);
        update_skin_texcoords(blendshapes_skin_gl_data_[i]);
        update_skin_weight_and_depends(blendshapes_skin_gl_data_[i]);
        update_skin_selection(blendshapes_skin_gl_data_[i]);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
set_selected_skin(int selected_skin)
{
    character_.set_selected_skin_idx(selected_skin);
    this->object_ = character_.skins()[character_.get_selected_skin_idx()];
}


//-----------------------------------------------------------------------------

int
Character_node::
set_selected_skin(const std::string &name)
{
    int idx = character_.get_skin_idx(name);
    if (idx != -1)
        set_selected_skin(idx);
    else
        set_selected_skin(0);
}

//-----------------------------------------------------------------------------



void
Character_node::
delete_selected_skin()
{
    if (character_.get_selected_skin_idx() < (int)skin_gl_data_.size())
    {
        size_t i;
        Skin_gl_data* sgd;
        character::Surface_mesh_skin* skin = NULL;

        const std::vector<Skin_gl_data*> sgd_old = skin_gl_data_;
        skin_gl_data_.clear();
        skin_gl_data_.reserve(sgd_old.size()-1);

        for (i=0; i < sgd_old.size(); ++i)
        {
            sgd = sgd_old[i];
            if ((int)i == character_.get_selected_skin_idx())
            {
                skin = sgd->skin_;
                delete sgd;
            }
            else
            {
                skin_gl_data_.push_back(sgd);
            }
        }

        for (unsigned int k = 0; k < character().blendshapes().size(); ++k)
        {
            if (skin == character_.blendshapes()[k]->base())
            {
                for (i=0; i < blendshapes_skin_gl_data_.size(); ++i)
                {
                    if (character_.blendshapes()[k] == blendshapes_skin_gl_data_[i]->blendshapes_)
                    {
                        blendshapes_skin_gl_data_.erase( blendshapes_skin_gl_data_.begin()+i );
                        break;
                    }
                }

                character().blendshapes().erase( character().blendshapes().begin()+k );
            }
        }

        if (skin != NULL)
            character_.delete_skin(skin);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
apply_blendshapes()
{
    for (unsigned int i = 0; i < character_.blendshapes().size(); ++i)
    {
        if (character_.blendshapes()[i]->apply())
        {
            update_skin_points(blendshapes_skin_gl_data_[i]);
            update_skin_normals(blendshapes_skin_gl_data_[i]);
        }
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
scale_to_height(float height, float ground_offset, bool set_on_ground, bool no_scale)
{
    bbox_ = character_.bbox();

    const float current_height = bbox_.max()[1] - bbox_.min()[1];
    float scale = height / current_height;

    if (no_scale)
    {
        scale = 1.0f;
    }

    Vec3f vec_to_ground(0.0f);

    if (set_on_ground)
    {
        vec_to_ground[1] = ground_offset - bbox_.min()[1];
    }


    Mat4f transform = Mat4f::scale(scale) * Mat4f::translate(vec_to_ground);

    apply_transform(transform);

    character::Skeleton& skeleton = character_.skeleton();
    character::Joint* joint;

    skeleton.root_->local_[12] += vec_to_ground[0];
    skeleton.root_->local_[13] += vec_to_ground[1];
    skeleton.root_->local_[14] += vec_to_ground[2];
    for (size_t i=0; i < skeleton.joints_.size(); ++i)
    {
        joint = skeleton.joints_[i];
        joint->local_[12] *= scale;
        joint->local_[13] *= scale;
        joint->local_[14] *= scale;
    }
    skeleton.init();
    character_.set_current_pose_as_bindpose();

    update_meshes();
}


//-----------------------------------------------------------------------------


void
Character_node::
scale(float scale)
{
    Mat4f transform = Mat4f::scale(scale);

    apply_transform(transform);

    character::Skeleton& skeleton = character_.skeleton();
    character::Joint* joint;

    for (size_t i=0; i < skeleton.joints_.size(); ++i)
    {
        joint = skeleton.joints_[i];
        joint->local_[12] *= scale;
        joint->local_[13] *= scale;
        joint->local_[14] *= scale;
    }
    skeleton.init();
    character_.set_current_pose_as_bindpose();

    update_meshes();
}


//-----------------------------------------------------------------------------


void
Character_node::
center()
{
    Skin_gl_data* sgd = skin_gl_data_[character_.get_selected_skin_idx()];
    Surface_mesh* mesh = sgd->skin_;

    auto points = mesh->vertex_property<Point>("v:point");

    Point cog(0.0f);

    for (auto v : mesh->vertices())
    {
        cog += points[v];
    }
    cog /= mesh->n_vertices();

    Mat4f transform = Mat4f::translate(-cog);

    apply_transform(transform);

    character::Skeleton& skeleton = character_.skeleton();
    character::Joint* joint;

    skeleton.root_->local_[12] -= cog[0];
    skeleton.root_->local_[13] -= cog[1];
    skeleton.root_->local_[14] -= cog[2];
    skeleton.init();
    character_.set_current_pose_as_bindpose();

    update_meshes();
}


//-----------------------------------------------------------------------------


void
Character_node::
draw(gl::GL_state *_gl)
{
    if (!visible_)
        return;


    if (new_data_)
    {
        update_skeleton();
        apply_blendshapes();
        new_data_ = false;
    }


    size_t i;
    Skin_gl_data* sgd;

    Vec3f front_selected_color = front_color_;
    if (is_target_)
        front_selected_color *= Vec3f(0.8f,1.0f,0.8f);

    gl::Shader *phong_shader          = _gl->set_active_shader(gl::PHONG_SHADER);
    phong_shader->set_uniform("model_matrix", _gl->model_);
    phong_shader->set_uniform("modelview_projection_matrix", _gl->modelviewproj_);
    phong_shader->set_uniform("modelview_matrix", _gl->modelview_);
    phong_shader->set_uniform("use_lighting",    false);
    phong_shader->set_uniform("use_texture1D",   false);
    phong_shader->set_uniform("use_texture2D",   false);
    phong_shader->set_uniform("use_vertexcolor", false);
    phong_shader->set_uniform("front_color",  front_color_);
    phong_shader->set_uniform("back_color",   back_color_);
    phong_shader->set_uniform("normal_matrix", _gl->normal_);

    gl::Shader *skinning_phong_shader = _gl->set_active_shader(gl::SKINNING_PHONG_SHADER);

    // shadow map stuff

    graphene::gl::Shadow_map* sm;

    if (_gl->use_shadowmaps_)
    {
        skinning_phong_shader->set_uniform("use_shadowmaps", true);

        skinning_phong_shader->set_uniform("strength", 0.75f);

        if (_gl->get_shadow_maps().size() > 0)
        {
            sm = _gl->get_shadow_maps()[0];
            skinning_phong_shader->bind_texture("shadowmap0", sm->get_depth_texture_id(), GL_TEXTURE_2D, 4);
            skinning_phong_shader->set_uniform("viewproj0", sm->viewproj_);
        }

        if (_gl->get_shadow_maps().size() > 1)
        {
            sm = _gl->get_shadow_maps()[1];
            skinning_phong_shader->bind_texture("shadowmap1", sm->get_depth_texture_id(), GL_TEXTURE_2D, 5);
            skinning_phong_shader->set_uniform("viewproj1", sm->viewproj_);
        }
    }
    else
    {
        skinning_phong_shader->set_uniform("use_shadowmaps", false);
    }


    // get current draw mode
    const std::string& draw_mode = get_draw_mode();

    // get current active settings for draw mode
    std::vector<std::string> draw_more_settings = get_draw_more_settings_active();


    //if the skeleton is empty, just use the phong
    if (character().skeleton().joints_.empty())
    {
        skinning_phong_shader = phong_shader;

        phong_shader->use();
    }

    else // skinning phong shader
    {
        skinning_phong_shader->use();

        skinning_phong_shader->set_uniform("use_lighting",    false);
        skinning_phong_shader->set_uniform("use_vertexcolor", false);
        skinning_phong_shader->set_uniform("use_texture1D",   false);
        skinning_phong_shader->set_uniform("use_texture2D",   false);
        skinning_phong_shader->set_uniform("front_color",  front_color_);
        skinning_phong_shader->set_uniform("back_color",   back_color_);
        skinning_phong_shader->set_uniform("model_matrix", _gl->model_);
        skinning_phong_shader->set_uniform("modelview_projection_matrix", _gl->modelviewproj_);
        skinning_phong_shader->set_uniform("modelview_matrix", _gl->modelview_);
        skinning_phong_shader->set_uniform("normal_matrix", _gl->normal_);

        skinning_phong_shader->bind_ubo("skinning_matrices", skeleton_gl_data_.ubo_id_, 0);
    }

    if (draw_mode == "Solid")
    {
        skinning_phong_shader->set_uniform("use_lighting",    true);

        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            if (i == (size_t)character_.get_selected_skin_idx())
                skinning_phong_shader->set_uniform("front_color", front_selected_color);
            else
                skinning_phong_shader->set_uniform("front_color", front_color_);

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
    }

    else if (draw_mode == "Solid Edge")
    {
        skinning_phong_shader->set_uniform("front_color",  wire_color_);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
        glDepthRange(0.002, 1.0);


        skinning_phong_shader->set_uniform("front_color",  front_color_);
        skinning_phong_shader->set_uniform("use_lighting", true);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            if (i == (size_t)character_.get_selected_skin_idx())
                skinning_phong_shader->set_uniform("front_color", front_selected_color);
            else
                skinning_phong_shader->set_uniform("front_color", front_color_);

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
        glDepthRange(0.0, 1.0);
    }

    if (draw_mode == "Vertex_color")
    {
        skinning_phong_shader->set_uniform("use_lighting", true);
        glDepthRange(0.002, 1.0);

        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            if (i == (size_t)character_.get_selected_skin_idx())
                skinning_phong_shader->set_uniform("front_color", front_selected_color);
            else
                skinning_phong_shader->set_uniform("front_color", front_color_);

            skinning_phong_shader->set_uniform("use_vertexcolor", (sgd->color_buffer_ != 0) );
            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
    }

    else if (draw_mode == "Wireframe")
    {
        skinning_phong_shader->set_uniform("front_color",  wire_color_);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            if (i == (size_t)character_.get_selected_skin_idx())
                skinning_phong_shader->set_uniform("front_color", front_selected_color);
            else
                skinning_phong_shader->set_uniform("front_color", wire_color_);

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    else if (draw_mode == "Textured (Shaded)")
    {
        bool with_normal_map = false;
        if (std::find(draw_more_settings.begin(), draw_more_settings.end(), "Normal Map") != draw_more_settings.end())
        {
            with_normal_map = true;
        }

        bool with_specular_map = false;
        if (std::find(draw_more_settings.begin(), draw_more_settings.end(), "Specular Map") != draw_more_settings.end())
        {
            with_specular_map = true;
        }

        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;

            if (sgd->texture_)
            {
                skinning_phong_shader->set_uniform("use_texture2D", true);
                skinning_phong_shader->set_uniform("use_lighting",  true);
                skinning_phong_shader->set_uniform("front_color", Vec3f(1.0f));
                skinning_phong_shader->bind_texture("texture2D",    sgd->texture_->id_, GL_TEXTURE_2D, 1);
            }
            else
            {
                skinning_phong_shader->set_uniform("use_texture2D", false);
                skinning_phong_shader->set_uniform("use_lighting",  true);
                skinning_phong_shader->set_uniform("front_color", front_color_);
            }


            if (with_normal_map)
            {
                skinning_phong_shader->bind_texture("texture2D_nm", sgd->texture_nm_->id_, GL_TEXTURE_2D, 2);
                skinning_phong_shader->set_uniform("use_normalmap", true);
            }
            else
            {
                skinning_phong_shader->set_uniform("use_normalmap", false);
            }

            if (with_specular_map)
            {
                skinning_phong_shader->bind_texture("texture2D_spec", sgd->texture_spec_->id_, GL_TEXTURE_2D, 3);
                skinning_phong_shader->set_uniform("use_specularmap", true);
            }
            else
            {
                skinning_phong_shader->set_uniform("use_specularmap", false);
            }

            if (sgd->skin_->alpha_ < 1.0f)
            {
                glEnable(GL_BLEND);
                skinning_phong_shader->set_uniform("alpha", sgd->skin_->alpha_);
                glBlendEquation(GL_FUNC_ADD);
                glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            }
            else
            {
                glDisable(GL_BLEND);
                skinning_phong_shader->set_uniform("alpha", 1.0f);
            }

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glEnable(GL_FRAMEBUFFER_SRGB);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
            glDisable(GL_FRAMEBUFFER_SRGB);
            glDisable(GL_BLEND);
        }
    }

    else if (draw_mode == "Textured")
    {
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;

            if (sgd->texture_)
            {
                skinning_phong_shader->set_uniform("use_texture2D", true);
                skinning_phong_shader->set_uniform("front_color", Vec3f(1.0f));
                skinning_phong_shader->bind_texture("texture2D",    sgd->texture_->id_, GL_TEXTURE_2D, 1);
            }
            else
            {
                skinning_phong_shader->set_uniform("use_texture2D", false);
                skinning_phong_shader->set_uniform("front_color", front_color_);
            }


            glBindVertexArray(sgd->vao_);
            glEnable(GL_FRAMEBUFFER_SRGB);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
            glDisable(GL_FRAMEBUFFER_SRGB);
        }
    }

    else if (draw_mode == "Textured Edge")
    {
        skinning_phong_shader->set_uniform("front_color",  wire_color_);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
        glDepthRange(0.002, 1.0);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;

            if (sgd->texture_)
            {
                skinning_phong_shader->set_uniform("use_texture2D", true);
                skinning_phong_shader->set_uniform("use_lighting",  true);
                skinning_phong_shader->set_uniform("front_color", Vec3f(1.0f));
                skinning_phong_shader->bind_texture("texture2D",    sgd->texture_->id_, GL_TEXTURE_2D, 1);
            }
            else
            {
                skinning_phong_shader->set_uniform("use_texture2D", false);
                skinning_phong_shader->set_uniform("use_lighting",  true);
                skinning_phong_shader->set_uniform("front_color", front_color_);
            }

            if (sgd->skin_->alpha_ < 1.0f)
            {
                glEnable(GL_BLEND);
                skinning_phong_shader->set_uniform("alpha", sgd->skin_->alpha_);
                glBlendEquation(GL_FUNC_ADD);
                glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            }
            else
            {
                glDisable(GL_BLEND);
                skinning_phong_shader->set_uniform("alpha", 1.0f);
            }

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glEnable(GL_FRAMEBUFFER_SRGB);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
            glDisable(GL_FRAMEBUFFER_SRGB);
        }
        glDepthRange(0.0, 1.0);
    }


    else if (draw_mode == "Skeleton")
    {

        skinning_phong_shader->set_uniform("use_lighting",    true);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            if (i == (size_t)character_.get_selected_skin_idx())
                skinning_phong_shader->set_uniform("front_color", front_selected_color);
            else
                skinning_phong_shader->set_uniform("front_color", front_color_);

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }

        glDisable(GL_DEPTH_TEST);

        phong_shader->use();
        phong_shader->set_uniform("use_lighting", false);
        phong_shader->set_uniform("front_color", wire_color_);

        glBindVertexArray(skeleton_gl_data_.bones_vao_);
        glDrawArrays(GL_LINES, 0, skeleton_gl_data_.n_bones_vertices_);

        phong_shader->set_uniform("use_vertexcolor", true);
        glBindVertexArray(skeleton_gl_data_.cs_vao_);
        glDrawArrays(GL_LINES, 0, skeleton_gl_data_.n_cs_vertices_);

        glEnable(GL_DEPTH_TEST);
    }

    else if (draw_mode == "Skeleton only")
    {
        glDisable(GL_DEPTH_TEST);

        phong_shader->use();
        phong_shader->set_uniform("use_lighting", false);
        phong_shader->set_uniform("front_color", Vec3f(0.0f));

        glBindVertexArray(skeleton_gl_data_.bones_vao_);
        glDrawArrays(GL_LINES, 0, skeleton_gl_data_.n_bones_vertices_);

        phong_shader->set_uniform("use_vertexcolor", true);
        glBindVertexArray(skeleton_gl_data_.cs_vao_);
        glDrawArrays(GL_LINES, 0, skeleton_gl_data_.n_cs_vertices_);

        glEnable(GL_DEPTH_TEST);
    }

    else if (draw_mode == "Points")
    {
        skinning_phong_shader->set_uniform("use_lighting",    true);
        glPointSize(4.0f);
        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (!sgd->skin_->visible_) continue;
            if (i == (size_t)character_.get_selected_skin_idx())
                skinning_phong_shader->set_uniform("front_color", front_selected_color);
            else
                skinning_phong_shader->set_uniform("front_color", front_color_);

            skinning_phong_shader->set_uniform("material",  sgd->skin_->material_);

            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_POINTS, 0, sgd->n_vertices_);
        }
        glPointSize(1.0f);
    }

    else if (draw_mode == "Unskinned")
    {
        gl::Shader *shader = _gl->set_active_shader(gl::PHONG_SHADER);
        shader->set_uniform("use_lighting", true);

        for (i=0; i < skin_gl_data_.size(); ++i)
        {
            sgd = skin_gl_data_[i];
            if (i == (size_t)character_.get_selected_skin_idx())
                shader->set_uniform("front_color", front_selected_color);
            else
                shader->set_uniform("front_color", front_color_);

            shader->set_uniform("material",  sgd->skin_->material_);


            glBindVertexArray(sgd->vao_);
            glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
        }
    }


    //draw selected points
    for (i=0; i < skin_gl_data_.size(); ++i)
    {
        draw_selection(_gl, skin_gl_data_[i]);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
draw_shadowmap(gl::GL_state *gls)
{
    if (!visible_)
        return;

    size_t i;
    Skin_gl_data* sgd;

    gl::Shader* shader = NULL;

    if (character_.skeleton().joints_.empty())
    {
        shader = gls->set_active_shader(gl::MESH_SHADOWMAP_SHADER);
    }
    else
    {
        shader = gls->set_active_shader(gl::SKINNING_SHADOWMAP_SHADER);
        shader->bind_ubo("skinning_matrices", skeleton_gl_data_.ubo_id_, 0);
    }

    shader->set_uniform("modelview_projection_matrix", gls->modelviewproj_);

    for (i=0; i < skin_gl_data_.size(); ++i)
    {
        sgd = skin_gl_data_[i];
        glBindVertexArray(sgd->vao_);
        glDrawArrays(GL_TRIANGLES, 0, sgd->n_vertices_);
    }
    glBindVertexArray(0);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skeleton()
{
    if ( character_.skeleton().joints_.empty() )
    {
        return;
    }

    character_.skeleton().update();
    //update ubo with new skeleton data
    glBindBuffer(GL_UNIFORM_BUFFER, skeleton_gl_data_.ubo_id_);
    glBufferData(GL_UNIFORM_BUFFER,
                 (unsigned int) character_.skeleton().skinning_matrices_.size()*sizeof(Mat4f),
                 character_.skeleton().skinning_matrices_.data(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);


    size_t i;
    character::Joint* joint;

    std::vector<Vec3f> bones_vertices;
    std::vector<Vec3f> cs_vertices;
    std::vector<Vec3f> cs_colors;


    bones_vertices.reserve(character_.skeleton().joints_.size() * 2);
    cs_vertices.reserve(character_.skeleton().joints_.size() * 6);
    cs_colors.reserve(character_.skeleton().joints_.size() * 6);

    const Vec3f x_ax(1.0f,0.0f,0.0f),y_ax(0.0f,1.0f,0.0f),z_ax(0.0f,0.0f,1.0f);

    for (i=0; i < character_.skeleton().joints_.size(); ++i)
    {
        joint = character_.skeleton().joints_[i];

        if (joint->parent_)
        {
            bones_vertices.push_back(joint->parent_->get_global_translation());
            bones_vertices.push_back(joint->get_global_translation());
        }

        cs_colors.push_back(x_ax);
        cs_colors.push_back(x_ax);
        cs_colors.push_back(y_ax);
        cs_colors.push_back(y_ax);
        cs_colors.push_back(z_ax);
        cs_colors.push_back(z_ax);

        const Mat3f rot = Mat3f(joint->global_);

        cs_vertices.push_back(joint->get_global_translation());
        cs_vertices.push_back(joint->get_global_translation() + normalize(rot * x_ax) * bbox_.size()*0.01f);
        cs_vertices.push_back(joint->get_global_translation());
        cs_vertices.push_back(joint->get_global_translation() + normalize(rot * y_ax) * bbox_.size()*0.01f);
        cs_vertices.push_back(joint->get_global_translation());
        cs_vertices.push_back(joint->get_global_translation() + normalize(rot * z_ax) * bbox_.size()*0.01f);
    }

    skeleton_gl_data_.n_bones_vertices_ = (unsigned int) bones_vertices.size();
    skeleton_gl_data_.n_cs_vertices_    = (unsigned int) cs_vertices.size();

    glBindVertexArray(skeleton_gl_data_.cs_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, skeleton_gl_data_.cs_vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, cs_vertices.size()*3*sizeof(float), cs_vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::VERTEX);
    glBindBuffer(GL_ARRAY_BUFFER, skeleton_gl_data_.cs_color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, cs_colors.size()*3*sizeof(float), cs_colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::COLOR, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::COLOR);

    glBindVertexArray(skeleton_gl_data_.bones_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, skeleton_gl_data_.bones_vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, bones_vertices.size()*3*sizeof(float), bones_vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::VERTEX);
}


//-----------------------------------------------------------------------------


void
Character_node::
init_skeleton_gl_data()
{
    if (character_.skeleton().joints_.empty())
    {
        return;
    }

    if (skeleton_gl_data_.cs_vao_ == 0)
        glGenVertexArrays(1, &skeleton_gl_data_.cs_vao_);
    if (skeleton_gl_data_.cs_vertex_buffer_ == 0)
        glGenBuffers(1, &skeleton_gl_data_.cs_vertex_buffer_);
    if (skeleton_gl_data_.cs_color_buffer_ == 0)
        glGenBuffers(1, &skeleton_gl_data_.cs_color_buffer_);

    if (skeleton_gl_data_.bones_vao_ == 0)
        glGenVertexArrays(1, &skeleton_gl_data_.bones_vao_);
    if (skeleton_gl_data_.bones_vertex_buffer_ == 0)
        glGenBuffers(1, &skeleton_gl_data_.bones_vertex_buffer_);

    if (skeleton_gl_data_.ubo_id_ == 0)
        glGenBuffers(1, &skeleton_gl_data_.ubo_id_);
}


//-----------------------------------------------------------------------------


void
Character_node::
init_skin_gl_data(Skin_gl_data *sgd, GL_state* gls)
{
    character::Surface_mesh_skin* skin = sgd->skin_;

    if (sgd->vao_ == 0)
        glGenVertexArrays(1,&sgd->vao_);

    if (sgd->vertex_buffer_ == 0 && skin->vertices_)
        glGenBuffers(1, &sgd->vertex_buffer_);

    if (sgd->normal_buffer_ == 0 && (skin->h_normals_ || skin->v_normals_))
        glGenBuffers(1, &sgd->normal_buffer_);

    if (sgd->texcoord_buffer_ == 0 && skin->h_texcoords_)
        glGenBuffers(1, &sgd->texcoord_buffer_);

    if (sgd->weight_buffer_ == 0 && skin->v_weights1_)
        glGenBuffers(1, &sgd->weight_buffer_);

    if (sgd->depend_buffer_ == 0 && skin->v_depends1_)
        glGenBuffers(1, &sgd->depend_buffer_);

    if (sgd->weight2_buffer_ == 0 && skin->v_weights2_)
        glGenBuffers(1, &sgd->weight2_buffer_);

    if (sgd->depend2_buffer_ == 0 && skin->v_depends2_)
        glGenBuffers(1, &sgd->depend2_buffer_);

    if (sgd->color_buffer_ == 0  && skin->v_colors_)
        glGenBuffers(1, &sgd->color_buffer_);

    if (sgd->selection_index_buffer_ == 0)
        glGenBuffers(1, &sgd->selection_index_buffer_);

    Surface_mesh::Mesh_property<std::string> texname = skin->get_mesh_property<std::string>("m:texturename");
    if (texname && !texname.vector().empty())
    {
        sgd->texture_ = gls->get_texture_from_heap(texname[0]);
    }
    Surface_mesh::Mesh_property<std::string> texname_nm = skin->get_mesh_property<std::string>("m:texturename_nm");
    if (texname_nm && !texname_nm.vector().empty())
    {
        sgd->texture_nm_ = gls->get_texture_from_heap(texname_nm[0]);
    }
    Surface_mesh::Mesh_property<std::string> texname_spec = skin->get_mesh_property<std::string>("m:texturename_spec");
    if (texname_spec && !texname_spec.vector().empty())
    {
        sgd->texture_spec_ = gls->get_texture_from_heap(texname_spec[0]);
    }

    update_skin_points(sgd);
    update_skin_normals(sgd);
    update_skin_texcoords(sgd);
    update_skin_texture(sgd);
    update_skin_weight_and_depends(sgd);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_points(Skin_gl_data *sgd)
{
    if (!sgd->skin_->vertices_)
        return;

    std::vector<Point> points;
    points.reserve(sgd->skin_->n_faces() * 3);

    surface_mesh::Surface_mesh::Vertex_property<Point> sm_points;
    sm_points = sgd->skin_->get_vertex_property<Point>("v:blended_point");
    surface_mesh::Surface_mesh::Vertex_property<size_t> sm_vindices;
    sm_vindices = sgd->skin_->vertex_property<size_t>("v:index");

    if (!sm_points)
        sm_points = sgd->skin_->vertices_;


    surface_mesh::Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    surface_mesh::Surface_mesh::Vertex v0, v1, v2;
    size_t i=0;
    for (auto f : sgd->skin_->faces())
    {
        fvit = fvend = sgd->skin_->vertices(f);
        v0 = *fvit; ++fvit;
        v2 = *fvit; ++fvit;
        do
        {
            v1 = v2;
            v2 = *fvit;

            points.push_back(sm_points[v0]);
            points.push_back(sm_points[v1]);
            points.push_back(sm_points[v2]);

            sm_vindices[v0] = i++;
            sm_vindices[v1] = i++;
            sm_vindices[v2] = i++;
        }
        while (++fvit != fvend);
    }

    glBindVertexArray(sgd->vao_);
    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, sgd->vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, points.size()*3*sizeof(float), points.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::VERTEX);
    sgd->n_vertices_ = (unsigned int)points.size();
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_colors(Skin_gl_data *sgd)
{
    if (!sgd->skin_->vertices_)
        return;

    Surface_mesh::Vertex_property<Color> vcolor = sgd->skin_->get_vertex_property<Color>("v:color");

    if (sgd->color_buffer_ == 0  && vcolor)
        glGenBuffers(1, &sgd->color_buffer_);

    if (!vcolor)
    {
        glBindVertexArray(sgd->vao_);
        glDisableVertexAttribArray(gl::attrib_locations::COLOR);
        return;
    }

    std::vector<Color> colors;
    colors.reserve(sgd->skin_->n_faces() * 3);

    surface_mesh::Surface_mesh::Vertex_property<Color> vcolors;
    vcolors = sgd->skin_->get_vertex_property<Color>("v:color");
    sgd->skin_->v_colors_ = vcolors;

    surface_mesh::Surface_mesh::Face_property<Color> fcolors;
    fcolors = sgd->skin_->get_face_property<Color>("f:color");

    surface_mesh::Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    surface_mesh::Surface_mesh::Vertex v0, v1, v2;
    for (auto f : sgd->skin_->faces())
    {
        fvit = fvend = sgd->skin_->vertices(f);
        v0 = *fvit; ++fvit;
        v2 = *fvit; ++fvit;
        do
        {
            v1 = v2;
            v2 = *fvit;

            if (vcolors)
            {
                colors.push_back(vcolors[v0]);
                colors.push_back(vcolors[v1]);
                colors.push_back(vcolors[v2]);
            }

            if (fcolors)
            {
                colors.push_back(fcolors[f]);
                colors.push_back(fcolors[f]);
                colors.push_back(fcolors[f]);
            }
        }
        while (++fvit != fvend);
    }

    glBindVertexArray(sgd->vao_);
    glBindBuffer(GL_ARRAY_BUFFER, sgd->color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, colors.size()*3*sizeof(float), colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::COLOR, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::COLOR);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_normals(Skin_gl_data *sgd)
{
    if (!sgd->skin_->h_normals_ && !sgd->skin_->v_normals_)
        return;

    std::vector<Normal> normals;
    normals.reserve(sgd->skin_->n_faces() * 3);


    surface_mesh::Surface_mesh::Vertex_property<Normal> sm_v_normals;
    sm_v_normals = sgd->skin_->get_vertex_property<Point>("v:blended_normal");
    if (!sm_v_normals)
        sm_v_normals = sgd->skin_->v_normals_;

    surface_mesh::Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    surface_mesh::Surface_mesh::Vertex v0, v1, v2;
    surface_mesh::Surface_mesh::Halfedge_around_face_circulator hfc_it,hfc_end;
    for (auto f : sgd->skin_->faces())
    {

        if (sgd->skin_->h_normals_)
        {
            //twaltema: gather normals for each generated/duplicated vertex from halfedge properties
            //ATTENTION: this does only work for triangle meshes
            hfc_it = hfc_end = sgd->skin_->halfedges(f);
            do
            {
                normals.push_back(sgd->skin_->h_normals_[*hfc_it]);
            }
            while (++hfc_it != hfc_end);
        }
        else if (sm_v_normals)
        {
            fvit = fvend = sgd->skin_->vertices(f);
            v0 = *fvit; ++fvit;
            v2 = *fvit; ++fvit;
            do
            {
                v1 = v2;
                v2 = *fvit;

                normals.push_back((sm_v_normals[v0]));
                normals.push_back((sm_v_normals[v1]));
                normals.push_back((sm_v_normals[v2]));
            }
            while (++fvit != fvend);
        }
    }

    glBindVertexArray(sgd->vao_);
    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, sgd->normal_buffer_);
    glBufferData(GL_ARRAY_BUFFER, normals.size()*3*sizeof(float), normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::NORMAL, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::NORMAL);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_texcoords(Skin_gl_data *sgd)
{
    if (!sgd->skin_->h_texcoords_)
        return;

    std::vector<Texture_coordinate> texcoords;
    texcoords.reserve(sgd->skin_->n_faces() * 3);

    surface_mesh::Surface_mesh::Halfedge_around_face_circulator hfc_it,hfc_end;
    for (auto f : sgd->skin_->faces())
    {
        //twaltema: gather texcoords for each generated/duplicated vertex from halfedge properties
        //ATTENTION: this does only work for triangle meshes
        hfc_it = hfc_end = sgd->skin_->halfedges(f);
        do
        {
            texcoords.push_back(sgd->skin_->h_texcoords_[*hfc_it]);
        }
        while (++hfc_it != hfc_end);
    }

    glBindVertexArray(sgd->vao_);
    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, sgd->texcoord_buffer_);
    glBufferData(GL_ARRAY_BUFFER, texcoords.size()*3*sizeof(float), texcoords.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::TEXCOORDS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::TEXCOORDS);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_weight_and_depends(Skin_gl_data *sgd)
{
    if (!sgd->skin_->vertices_ || !sgd->skin_->v_weights1_ || !sgd->skin_->v_depends1_ || !sgd->skin_->v_weights2_ || !sgd->skin_->v_depends2_)
        return;

    std::vector<Vec4f> weights;
    std::vector<Vec4f> depends;
    std::vector<Vec4f> weights2;
    std::vector<Vec4f> depends2;
    weights.reserve(sgd->skin_->n_faces() * 3);
    depends.reserve(sgd->skin_->n_faces() * 3);
    weights2.reserve(sgd->skin_->n_faces() * 3);
    depends2.reserve(sgd->skin_->n_faces() * 3);


    surface_mesh::Surface_mesh::Vertex_around_face_circulator fvit, fvend;
    surface_mesh::Surface_mesh::Vertex v0, v1, v2;
    for (auto f : sgd->skin_->faces())
    {
        fvit = fvend = sgd->skin_->vertices(f);
        v0 = *fvit; ++fvit;
        v2 = *fvit; ++fvit;
        do
        {
            v1 = v2;
            v2 = *fvit;

            weights.push_back(sgd->skin_->v_weights1_[v0]);
            weights.push_back(sgd->skin_->v_weights1_[v1]);
            weights.push_back(sgd->skin_->v_weights1_[v2]);

            depends.push_back(sgd->skin_->v_depends1_[v0]);
            depends.push_back(sgd->skin_->v_depends1_[v1]);
            depends.push_back(sgd->skin_->v_depends1_[v2]);

            weights2.push_back(sgd->skin_->v_weights2_[v0]);
            weights2.push_back(sgd->skin_->v_weights2_[v1]);
            weights2.push_back(sgd->skin_->v_weights2_[v2]);

            depends2.push_back(sgd->skin_->v_depends2_[v0]);
            depends2.push_back(sgd->skin_->v_depends2_[v1]);
            depends2.push_back(sgd->skin_->v_depends2_[v2]);

        }
        while (++fvit != fvend);
    }

    glBindVertexArray(sgd->vao_);
    glBindBuffer(GL_ARRAY_BUFFER, sgd->weight_buffer_);
    glBufferData(GL_ARRAY_BUFFER, weights.size()*4*sizeof(float), weights.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::WEIGHT, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::WEIGHT);

    glBindBuffer(GL_ARRAY_BUFFER, sgd->depend_buffer_);
    glBufferData(GL_ARRAY_BUFFER, depends.size()*4*sizeof(float), depends.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::DEPEND, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::DEPEND);


    glBindBuffer(GL_ARRAY_BUFFER, sgd->weight2_buffer_);
    glBufferData(GL_ARRAY_BUFFER, weights2.size()*4*sizeof(float), weights2.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::WEIGHT2, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::WEIGHT2);

    glBindBuffer(GL_ARRAY_BUFFER, sgd->depend2_buffer_);
    glBufferData(GL_ARRAY_BUFFER, depends2.size()*4*sizeof(float), depends2.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(gl::attrib_locations::DEPEND2, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(gl::attrib_locations::DEPEND2);

}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_texture(Skin_gl_data *sgd)
{
    if (sgd->texture_ == NULL)
        return;

    if (sgd->texture_->id_ == 0) // albedo map
    {
        glGenTextures(1,&sgd->texture_->id_);
    }


    // albedo map
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, sgd->texture_->id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, sgd->texture_->width_, sgd->texture_->height_,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, sgd->texture_->data_.data());


    if (sgd->texture_nm_ != NULL)
    {
        if (sgd->texture_nm_->id_ == 0) // normal map
        {
            glGenTextures(1,&sgd->texture_nm_->id_);
        }

        // normal map
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, sgd->texture_nm_->id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sgd->texture_nm_->width_, sgd->texture_nm_->height_,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, sgd->texture_nm_->data_.data());

        add_draw_more_settings("Normal Map");
    }


    if (sgd->texture_spec_ != NULL)
    {
        if (sgd->texture_spec_->id_ == 0) // specular map
        {
            glGenTextures(1,&sgd->texture_spec_->id_);
        }

        // specular map
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, sgd->texture_spec_->id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, sgd->texture_spec_->width_, sgd->texture_spec_->height_,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, sgd->texture_spec_->data_.data());

        add_draw_more_settings("Specular Map");
    }

    glActiveTexture(GL_TEXTURE0);
}


//-----------------------------------------------------------------------------


void
Character_node::
update_skin_selection(Skin_gl_data *sgd)
{
    Surface_mesh::Vertex_iterator v_it;
    Surface_mesh* mesh = sgd->skin_;
    Surface_mesh::Vertex_property<bool> vselected = mesh->get_vertex_property<bool>("v:selected");
    Surface_mesh::Vertex_property<size_t> vindices = mesh->get_vertex_property<size_t>("v:index");
    std::vector<unsigned int> selection;

    if (vselected && vindices)
    {
        for (v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
        {
            if (vselected[*v_it])
            {
                selection.push_back((unsigned int)vindices[*v_it]);
            }
        }
    }
    sgd->n_selected_ = (GLsizei)selection.size();
    if (sgd->n_selected_ > 0)//otherwise it will crash on Windows
    {
        glBindVertexArray(sgd->vao_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sgd->selection_index_buffer_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, selection.size()*sizeof(unsigned int), &selection[0], GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
recompute_normals(Skin_gl_data *sgd)
{
    if (sgd->blendshapes_ != NULL)
    {
        size_t i;
        character::Blendshapes::Target_shape* target;
        for (i=0; i < sgd->blendshapes_->targets().size(); ++i)
        {
            target = sgd->blendshapes_->targets()[i];
            surface_mesh::crease_normals(target, crease_angle_, "h:normal");
            target->init_properties();
        }
    }

    surface_mesh::crease_normals(sgd->skin_, crease_angle_, "h:normal");
    sgd->skin_->init_properties();
}


//-----------------------------------------------------------------------------


void
Character_node::
draw_selection(gl::GL_state *_gl, Skin_gl_data *sgd)
{
    if (sgd->n_selected_ > 0)
    {
        gl::Shader* skinning_phong_shader = _gl->set_active_shader(gl::SKINNING_PHONG_SHADER);

        glDepthRange(0.0, 1.0);
        glPointSize(5.0);

        skinning_phong_shader->set_uniform("use_lighting",    true);
        skinning_phong_shader->set_uniform("use_vertexcolor", false);
        skinning_phong_shader->set_uniform("use_texture2D",     false);
        skinning_phong_shader->set_uniform("use_texture1D",     false);
        skinning_phong_shader->set_uniform("front_color",     Vec3f(1.0f, 0.0f, 0.0f));
        skinning_phong_shader->set_uniform("material",        Vec4f(0.1f, 1.0f, 0.0f, 0.0f));
        skinning_phong_shader->set_uniform("modelview_projection_matrix", _gl->modelviewproj_);
        skinning_phong_shader->set_uniform("modelview_matrix", _gl->modelview_);
        skinning_phong_shader->set_uniform("normal_matrix", _gl->normal_);

        glBindVertexArray(sgd->vao_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sgd->selection_index_buffer_);
        glDrawElements(GL_POINTS, sgd->n_selected_, GL_UNSIGNED_INT, NULL);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


        if (sgd->selection_highlight_idx_ != -1)
        {
            glPointSize(10.0);
            skinning_phong_shader->set_uniform("front_color",     Vec3f(0.0f, 1.0f, 0.0f));
            glDrawArrays(GL_POINTS, sgd->selection_highlight_idx_, 1);
        }
    }
}


//-----------------------------------------------------------------------------


void
Character_node::
reset_to_bindpose()
{
    character_.skeleton().reset_to_bindpose();
    update_skeleton();
}


//-----------------------------------------------------------------------------


std::vector< gl::Texture* >
Character_node::
get_textures() const
{
    std::set< gl::Texture* > textures_set;

    for (unsigned int i = 0; i < skin_gl_data_.size(); ++i)
    {
        if (skin_gl_data_[i]->texture_)
        {
            textures_set.insert(skin_gl_data_[i]->texture_);
        }

        if (skin_gl_data_[i]->texture_nm_)
        {
            textures_set.insert(skin_gl_data_[i]->texture_nm_);
        }

        if (skin_gl_data_[i]->texture_spec_)
        {
            textures_set.insert(skin_gl_data_[i]->texture_spec_);
        }
    }

    std::vector< gl::Texture* > textures_vec(textures_set.begin(), textures_set.end());
    
    return textures_vec;
}


//-----------------------------------------------------------------------------


void
Character_node::
update_textures()
{
    for (unsigned int i = 0; i < skin_gl_data_.size(); ++i)
    {
        update_skin_texture(skin_gl_data_[i]);
    }
}


//=============================================================================
} //namespace scene_graph
} //namespace graphene
//=============================================================================

