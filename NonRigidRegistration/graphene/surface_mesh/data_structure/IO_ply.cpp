//=============================================================================
// Copyright (C) 2001-2005 by Computer Graphics Group, RWTH Aachen
// Copyright (C) 2011 by Graphics & Geometry Group, Bielefeld University
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public License
// as published by the Free Software Foundation, version 2.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//=============================================================================


//== INCLUDES =================================================================

#include "IO.h"

#include <sstream>
#include <fstream>


#include "ply/ply.h"

#define NUM_PROPS 6

//== NAMESPACES ===============================================================


namespace graphene {
namespace surface_mesh {


typedef struct Vertex {
  float x,y,z;
  float nx,ny,nz;
  void *other_props;       /* other properties */
} Vertex;


typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
} Face;

bool read_ply(Surface_mesh &mesh, const std::string &filename)
{
    FILE* in;
    PlyFile* ply;
    PlyElement* ply_elem;
    PlyProperty* ply_prop;
    PlyOtherProp *vert_other,*face_other;
    std::string str;
    int i,j,k;
    int elem_count;
    char* elem_name;
    Vertex v;
    Face f;

    bool has_prop[NUM_PROPS];
    for (i=0; i < NUM_PROPS; ++i)
        has_prop[i] = false;

    PlyProperty vert_props[NUM_PROPS] = { /* list of property information for a vertex */
      {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
      {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
      {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
      {"nx", Float32, Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
      {"ny", Float32, Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
      {"nz", Float32, Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
    };


    PlyProperty face_prop = /* list of property information for a face */
      {"vertex_indices", Int32, Int32, offsetof(Face,verts),
       1, Uint8, Uint8, offsetof(Face,nverts)};

    Surface_mesh::Vertex sm_vert;
    Surface_mesh::Vertex_property<Normal> normals;
    std::vector<Surface_mesh::Vertex> vertices;

    in = fopen(filename.c_str(), "r");

    ply = read_ply(in);

    if (ply == NULL)
    {
        std::cerr << "read_ply: [ERROR] Could not read ply file. Please make sure file has unix line endings!" << std::endl;
        return false;
    }

    for (i=0; i < ply->num_elem_types; ++i)
    {
        elem_name = setup_element_read_ply(ply, i, &elem_count);
        ply_elem = ply->elems[i];

        str = elem_name;

        if (str == "vertex")
        {
            mesh.reserve(elem_count, 3*elem_count, 2*elem_count);

            for (j = 0; j < ply_elem->nprops; j++)
            {
                ply_prop = ply_elem->props[j];
                str = ply_prop->name;

                for (k=0; k < NUM_PROPS; ++k)
                {
                    PlyProperty& prop = vert_props[k];
                    if (str == prop.name)
                    {
                        setup_property_ply(ply, &prop);
                        has_prop[k] = true;
                    }
                }
            }
            vert_other = get_other_properties_ply(ply, offsetof(Vertex, other_props));

            if (has_prop[3] && has_prop[4] && has_prop[5])
            {
                normals = mesh.vertex_property<Normal>("v:normal");
            }

            for (j=0; j < elem_count; ++j)
            {
                get_element_ply(ply, (void*) &v);
                sm_vert = mesh.add_vertex(Point(v.x,v.y,v.z));

                if (normals)
                {
                    normals[sm_vert] = Normal(v.nx,v.ny,v.nz);
                }
            }

        }
        else if (str == "face")
        {
            setup_property_ply(ply, &face_prop);
            face_other = get_other_properties_ply(ply, offsetof(Face, other_props));

            for (j=0; j < elem_count; ++j)
            {
                get_element_ply(ply, &f);
                const int val = (int) f.nverts;
                vertices.clear();
                for (k=0; k < val; ++k)
                {
                    vertices.push_back(Surface_mesh::Vertex(f.verts[k]));
                }
                mesh.add_face(vertices);
            }
        }
    }

    return true;
}

//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================
