//== INCLUDES =================================================================

#include "IO.h"
#include <stdio.h>


//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


// helper function
template <typename T> size_t read(FILE* in, T& t)
{
    return fread((char*)&t, 1, sizeof(t), in);
}


// helper function
template <typename T> size_t write(FILE* out, T& t)
{
    return fwrite((char*)&t, 1, sizeof(t), out);
}


//-----------------------------------------------------------------------------


bool read_poly(Surface_mesh& mesh, const std::string& filename)
{
    unsigned int n_items;

    // open file (in binary mode)
    FILE* in = fopen(filename.c_str(), "rb");
    if (!in) return false;


    // clear mesh
    mesh.clear();


    // how many elements?
    unsigned int nv, ne, nh, nf;
    bool has_colors;
    read(in, nv);
    read(in, ne);
    read(in, nf);
    read(in, has_colors);
    nh = 2*ne;


    // resize containers
    mesh.vprops_.resize(nv);
    mesh.hprops_.resize(nh);
    mesh.eprops_.resize(ne);
    mesh.fprops_.resize(nf);


    // get properties
    auto vconn = mesh.vertex_property<Surface_mesh::Vertex_connectivity>("v:connectivity");
    auto hconn = mesh.halfedge_property<Surface_mesh::Halfedge_connectivity>("h:connectivity");
    auto fconn = mesh.face_property<Surface_mesh::Face_connectivity>("f:connectivity");
    auto point = mesh.vertex_property<Point>("v:point");


    // read properties from file
    n_items = fread((char*)vconn.data(), sizeof(Surface_mesh::Vertex_connectivity),   nv, in);
    n_items = fread((char*)hconn.data(), sizeof(Surface_mesh::Halfedge_connectivity), nh, in);
    n_items = fread((char*)fconn.data(), sizeof(Surface_mesh::Face_connectivity),     nf, in);
    n_items = fread((char*)point.data(), sizeof(Point),                               nv, in);


    if (has_colors)
    {
        auto color = mesh.vertex_property<Color>("v:color");
        n_items = fread((char*)color.data(), sizeof(Color), nv, in);
    }


    fclose(in);
    return true;
}


//-----------------------------------------------------------------------------


bool write_poly(const Surface_mesh& mesh, const std::string& filename)
{
    unsigned int n_items;


    // check for colors
    auto color = mesh.get_vertex_property<Color>("v:color");
    bool has_colors = color;


    // open file (in binary mode)
    FILE* out = fopen(filename.c_str(), "wb");
    if (!out) return false;


    // how many elements?
    unsigned int nv, ne, nh, nf;
    nv = mesh.n_vertices();
    ne = mesh.n_edges();
    nh = mesh.n_halfedges();
    nf = mesh.n_faces();

    write(out, nv);
    write(out, ne);
    write(out, nf);
    write(out, has_colors);
    nh = 2*ne;


    // get properties
    Surface_mesh::Vertex_property<Surface_mesh::Vertex_connectivity>      vconn = mesh.get_vertex_property<Surface_mesh::Vertex_connectivity>("v:connectivity");
    Surface_mesh::Halfedge_property<Surface_mesh::Halfedge_connectivity>  hconn = mesh.get_halfedge_property<Surface_mesh::Halfedge_connectivity>("h:connectivity");
    Surface_mesh::Face_property<Surface_mesh::Face_connectivity>          fconn = mesh.get_face_property<Surface_mesh::Face_connectivity>("f:connectivity");
    Surface_mesh::Vertex_property<Point>                                  point = mesh.get_vertex_property<Point>("v:point");


    // write properties to file
    fwrite((char*)vconn.data(), sizeof(Surface_mesh::Vertex_connectivity),   nv, out);
    fwrite((char*)hconn.data(), sizeof(Surface_mesh::Halfedge_connectivity), nh, out);
    fwrite((char*)fconn.data(), sizeof(Surface_mesh::Face_connectivity),     nf, out);
    fwrite((char*)point.data(), sizeof(Point),                               nv, out);

    if (has_colors) fwrite((char*)color.data(), sizeof(Color), nv, out);

    fclose(out);

    return true;
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
