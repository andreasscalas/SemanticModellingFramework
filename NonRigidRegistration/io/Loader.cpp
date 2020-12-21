
#include "Loader.h"
#include <stb_image/stb_image.h>

using namespace graphene;

namespace io
{

Loader::Loader(gl::GL_state &gl_state, scene_graph::Scene_graph &scene_graph) :
    gl_state_(gl_state),
    scene_graph_(scene_graph)
{

}

Loader::~Loader()
{

}


bool Loader::load(const std::vector<std::string> &filenames)
{
    for (auto filename:filenames)
    {
        if (! load(filename))
        {
            std::cout << "Loader::load: [ERROR] Could not load file: " << filename << std::endl;
            return false;
        }
    }
    return true;
}

bool Loader::load(const std::string &filename)
{
    const std::string ext = filename.substr(filename.rfind('.')+1);

    if (ext == "bim")
    {
        scene_graph::Character_node* cnode = new scene_graph::Character_node(filename);
        if (cnode->load(filename))
        {
            character::Surface_mesh_skin* skin;
            for (size_t i=0; i < cnode->character().skins().size(); ++i)
            {
                skin = cnode->character().skins()[i];
                load_textures(*skin);
            }

            scene_graph_.add_node(cnode);
            std::cout << "Loader::load: [STATUS] Loaded \"" << filename << "\"." << std::endl;
            return true;
        }
        else
        {
            std::cerr << "Loader::load: [ERROR] Unable to load \"" << filename << "\"." << std::endl;
            delete cnode;
            return false;
        }
        return true;
    }
    else if (ext == "txt" || ext == "cxyz" || ext == "xyz")
    {
        scene_graph::Point_set_node* psnode = new scene_graph::Point_set_node(filename);
        if (psnode->load(filename))
        {
            scene_graph_.add_node(psnode);
            std::cout << "Loader::load: [STATUS] Loaded \"" << filename << "\"." << std::endl;
            return true;
        }
        else
        {
            std::cerr << "Loader::load: [ERROR] Unable to load \"" << filename << "\"." << std::endl;
            delete psnode;
            return false;
        }
    }
    else if (ext == "obj" || ext == "off")
    {
        scene_graph::Surface_mesh_node *snode = new scene_graph::Surface_mesh_node(filename);
        if (snode->load(filename))
        {
            scene_graph_.add_node(snode);
            std::cout << "Loader::load: [STATUS] Loaded \"" << filename << "\"." << std::endl;
            return true;
        }
        else
        {
            std::cerr << "Loader::load: [ERROR] Unable to load \"" << filename << "\"." << std::endl;
            delete snode;
            return false;
        }
    }
    else //unknown file extension
    {
        std::cerr << "Loader::load: [ERROR] Unknown file extension of file \"" << filename << "\"." << std::endl;
        return false;
    }
}


bool Loader::load_textures(const surface_mesh::Surface_mesh &mesh)
{

    surface_mesh::Surface_mesh::Mesh_property<std::string> texname = mesh.get_mesh_property<std::string>("m:texturename");
    if (texname)
    {
        const std::string& texfilename = texname[0];

        //if texture already loaded, it should be on the heap
        gl::Texture* texture = gl_state_.get_texture_from_heap(texfilename);
        //if not, load it
        if (texture == nullptr)
        {
            texture = new gl::Texture;

            if (texture->read(texfilename))
            {
                //and add to texture heap
                gl_state_.add_texture_to_heap(texture);
            }
            else
            {
                delete texture;
            }
        }
    }

    return true;

    //TODO: do not load normal and specular map for now, since we do not use/need them anyway
/*
    surface_mesh::Surface_mesh::Mesh_property<std::string> texname_nm = pnode->mesh().get_mesh_property<std::string>("m:texturename_nm");
    if (texname_nm)
    {
        const std::string& texfilename = texname_nm[0];
        //if texture already loaded, it should be on the heap
        gl::Texture* texture = main_window_->qglviewer_->get_GL_state()->get_texture_from_heap(texfilename);
        //if not, load it
        if (texture == NULL)
        {
            QImage qimg;
            qimg.load(texfilename.c_str());
            if (!qimg.isNull())
            {
                //convert to gl compatible format
                qimg = QGLWidget::convertToGLFormat(qimg);

                //allocate and copy data
                unsigned char* data = new unsigned char[qimg.byteCount()];
                memcpy(data, qimg.bits(), qimg.byteCount()*sizeof(unsigned char));
                //create new texture
                texture = new gl::Texture(texfilename, qimg.width(), qimg.height(), data);
                //and add to texture heap
                main_window_->qglviewer_->get_GL_state()->add_texture_to_heap(texture);
            }
        }
        //set texture for new Surface_mesh_node
        pnode->set_texture_normalmap(texture);
    }

    surface_mesh::Surface_mesh::Mesh_property<std::string> texname_spec = pnode->mesh().get_mesh_property<std::string>("m:texturename_spec");
    if (texname_spec)
    {
        const std::string& texfilename = texname_spec[0];
        //if texture already loaded, it should be on the heap
        gl::Texture* texture = main_window_->qglviewer_->get_GL_state()->get_texture_from_heap(texfilename);
        //if not, load it
        if (texture == NULL)
        {
            QImage qimg;
            qimg.load(texfilename.c_str());
            if (!qimg.isNull())
            {
                //convert to gl compatible format
                qimg = QGLWidget::convertToGLFormat(qimg);

                //allocate and copy data
                unsigned char* data = new unsigned char[qimg.byteCount()];
                memcpy(data, qimg.bits(), qimg.byteCount()*sizeof(unsigned char));
                //create new texture
                texture = new gl::Texture(texfilename, qimg.width(), qimg.height(), data);
                //and add to texture heap
                main_window_->qglviewer_->get_GL_state()->add_texture_to_heap(texture);
            }
        }
        //set texture for new Surface_mesh_node
        pnode->set_texture_specularmap(texture);
    }
    */
}


} //namespace io
