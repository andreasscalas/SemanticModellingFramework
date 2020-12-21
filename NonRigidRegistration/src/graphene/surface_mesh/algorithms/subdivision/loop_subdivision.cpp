//== INCLUDES =================================================================

#include <graphene/surface_mesh/algorithms/subdivision/loop_subdivision.h>


//== NAMESPACE ================================================================

namespace graphene {
namespace surface_mesh {


//== IMPLEMENTATION ==========================================================


void loop_subdivision(Surface_mesh& mesh)
{
    if (!mesh.is_triangle_mesh())
    {
        return;
    }


    // reserve memory
    int nv = mesh.n_vertices();
    int ne = mesh.n_edges();
    int nf = mesh.n_faces();
    mesh.reserve(nv+ne, 2*ne+3*nf, 4*nf);


    // get properties
    auto points = mesh.vertex_property<Point>("v:point");
    auto vpoint = mesh.add_vertex_property<Point>("loop:vpoint");
    auto epoint = mesh.add_edge_property<Point>("loop:epoint");
    auto vfeature = mesh.get_vertex_property<bool>("v:feature");
    auto efeature = mesh.get_edge_property<bool>("e:feature");
    auto texcoords = mesh.get_halfedge_property<Texture_coordinate>("h:texcoord");
    auto etexcoords = mesh.add_halfedge_property<Texture_coordinate>("loop:etexcoord");
    auto hnormals   = mesh.get_halfedge_property<Normal>("h:normal");
    auto vselected  = mesh.get_vertex_property<bool>("v:selected");
    auto fselected  = mesh.add_face_property<bool>("f:selected");
    auto vomit      = mesh.get_vertex_property<bool>("v:omit");
    auto vfitting        = mesh.get_vertex_property<double>("v:fitting_weight");
    auto vlandmark       = mesh.get_vertex_property<double>("v:landmark_weight");
    auto vregularization = mesh.get_vertex_property<double>("v:regularization_weight");
    auto vdepends = mesh.get_vertex_property<Vec4f>("v:skin_depend");
    auto vweights = mesh.get_vertex_property<Vec4f>("v:skin_weight");


    if (vselected)
    {
        for (auto f : mesh.faces())
        {
            auto fvit = mesh.vertices(f);
            const auto v0 = *fvit;
            const auto v1 = *(++fvit);
            const auto v2 = *(++fvit);

            if (vselected[v0] && vselected[v1] && vselected[v2])
            {
                fselected[f] = true;
            }
            else
            {
                fselected[f] = false;
            }
        }
    }


    // set texture coordinates on boundary
    if (texcoords)
    {
        for (auto e: mesh.edges())
        {
            Surface_mesh::Halfedge h0 = mesh.halfedge(e, 0);
            Surface_mesh::Halfedge h1 = mesh.halfedge(e, 1);

            if (mesh.is_boundary(h1))
            {
                Surface_mesh::Halfedge h0p = mesh.prev_halfedge(h0);
                texcoords[h1] = texcoords[h0p];
            }
        }
    }


    // compute vertex positions
    for (auto v: mesh.vertices())
    {
        // isolated vertex?
        if (mesh.is_isolated(v))
        {
            vpoint[v] = points[v];
        }

        // boundary vertex?
        else if (mesh.is_boundary(v))
        {
            Surface_mesh::Halfedge h1 = mesh.halfedge(v);
            Surface_mesh::Halfedge h0 = mesh.prev_halfedge(h1);

            Point p = points[v];
            p *= 6.0;
            p += points[mesh.to_vertex(h1)];
            p += points[mesh.from_vertex(h0)];
            p *= 0.125;

            vpoint[v] = p;
        }

        // interior feature vertex?
        else if (vfeature && vfeature[v])
        {
            Surface_mesh::Halfedge h1 = mesh.halfedge(v);
            Surface_mesh::Halfedge h0 = mesh.prev_halfedge(h1);

            Point p = points[v];
            p *= 6.0;

            int count(0);
            for (auto h: mesh.halfedges(v))
            {
                if (efeature[mesh.edge(h)])
                {
                    Surface_mesh::Vertex vv = mesh.to_vertex(h);
                    p += points[vv];

                    ++count;
                }
            }

            if (count == 2) // vertex is on feature edge
            {
                p *= 0.125;
                vpoint[v] = p;
            }
            else // keep fixed
            {
                vpoint[v] = points[v];
            }
        }

        // interior vertex
        else
        {
            Point   p(0,0,0);
            Scalar  k(0);

            for (auto vv: mesh.vertices(v))
            {
                p += points[vv];

                ++k;
            }
            p /= k;

            Scalar  beta = (0.625 - pow(0.375 + 0.25*cos(2.0*M_PI/k), 2.0));

            vpoint[v] = points[v]*(Scalar)(1.0-beta) + beta*p;
        }
    }


    // compute edge positions
    for (auto e: mesh.edges())
    {
        // boundary or feature edge?
        if (mesh.is_boundary(e) || (efeature && efeature[e]))
        {
            epoint[e] = (points[mesh.vertex(e,0)] + points[mesh.vertex(e,1)]) * Scalar(0.5);

            if (texcoords)
            {
                Surface_mesh::Halfedge h0  = mesh.halfedge(e, 0);
                Surface_mesh::Halfedge h0p = mesh.prev_halfedge(h0);
                Surface_mesh::Halfedge h1  = mesh.halfedge(e, 1);
                Surface_mesh::Halfedge h1p = mesh.prev_halfedge(h1);

                etexcoords[h0]             = (texcoords[h0] + texcoords[h0p]) * Scalar(0.5);
                etexcoords[h1]             = (texcoords[h1] + texcoords[h1p]) * Scalar(0.5);
            }
        }

        // interior edge
        else
        {
            Surface_mesh::Halfedge h0 = mesh.halfedge(e, 0);
            Surface_mesh::Halfedge h1 = mesh.halfedge(e, 1);
            Point p = points[mesh.to_vertex(h0)];
            p += points[mesh.to_vertex(h1)];
            p *= 3.0;
            p += points[mesh.to_vertex(mesh.next_halfedge(h0))];
            p += points[mesh.to_vertex(mesh.next_halfedge(h1))];
            p *= 0.125;
            epoint[e] = p;

            if (texcoords)
            {
                Surface_mesh::Halfedge h0p = mesh.prev_halfedge(h0);
                Surface_mesh::Halfedge h1p = mesh.prev_halfedge(h1);

                etexcoords[h0] = (texcoords[h0] + texcoords[h0p]) * Scalar(0.5);
                etexcoords[h1] = (texcoords[h1] + texcoords[h1p]) * Scalar(0.5);
            }
        }
    }


    // set new vertex positions
    for (auto v: mesh.vertices())
    {
        if (vselected)
        {
            if (vselected[v])
            {
                points[v] = vpoint[v];
            }
        }
        else
        {
            points[v] = vpoint[v];
        }
    }


    // insert new vertices on edges
    for (auto e: mesh.edges())
    {
        if (vselected)
        {
            Surface_mesh::Halfedge e0 = mesh.halfedge(e, 0);
            Surface_mesh::Halfedge e1 = mesh.halfedge(e, 1);
            Surface_mesh::Face f0 = mesh.face(e0);
            Surface_mesh::Face f1 = mesh.face(e1);

            bool split_edge = false;
            if (mesh.is_valid(f0) && fselected[f0])
            {
                split_edge = true;
            }
            if (mesh.is_valid(f1) && fselected[f1])
            {
                split_edge = true;
            }

            if (!split_edge)
            {
                continue;
            }
        }

        Surface_mesh::Halfedge o1  = mesh.insert_vertex(e, epoint[e]);

        if (vomit)
        {
            vomit[mesh.to_vertex(o1)] = vomit[mesh.from_vertex(o1)] && vomit[mesh.to_vertex(mesh.next_halfedge(o1))];
        }

        if (vfitting)
        {
            vfitting[mesh.to_vertex(o1)] = ( vfitting[mesh.from_vertex(o1)] + vfitting[mesh.to_vertex(mesh.next_halfedge(o1))] ) * Scalar(0.5);
        }
        if (vlandmark)
        {
            vlandmark[mesh.to_vertex(o1)] = ( vlandmark[mesh.from_vertex(o1)] + vlandmark[mesh.to_vertex(mesh.next_halfedge(o1))] ) * Scalar(0.5);
        }
        if (vregularization)
        {
            vregularization[mesh.to_vertex(o1)] = ( vregularization[mesh.from_vertex(o1)] + vregularization[mesh.to_vertex(mesh.next_halfedge(o1))] ) * Scalar(0.5);
        }

        if (vdepends)
        {
            Surface_mesh::Vertex v  = mesh.to_vertex(o1);
            Surface_mesh::Vertex v0 = mesh.to_vertex(mesh.next_halfedge(o1));
            vdepends[v] = vdepends[v0];
        }

        if (vweights)
        {
            Surface_mesh::Vertex v  = mesh.to_vertex(o1);
            Surface_mesh::Vertex v0 = mesh.to_vertex(mesh.next_halfedge(o1));
            vweights[v] = vweights[v0];
        }

        // feature edge?
        if (efeature && efeature[e])
        {
            Surface_mesh::Vertex   v  = mesh.to_vertex(o1);
            Surface_mesh::Edge     e0 = mesh.edge(o1);
            Surface_mesh::Edge     e1 = mesh.edge(mesh.next_halfedge(o1));
            vfeature[v]  = true;
            efeature[e0] = true;
            efeature[e1] = true;
        }

        if (texcoords)
        {
            Surface_mesh::Halfedge h1 = mesh.opposite_halfedge(o1);
            Surface_mesh::Halfedge h0 = mesh.prev_halfedge(h1);
            Surface_mesh::Halfedge o0 = mesh.opposite_halfedge(h0);

            texcoords[h1] = texcoords[h0];
            texcoords[h1] = texcoords[h0];

            etexcoords[h1] = etexcoords[h0];
            etexcoords[o1] = etexcoords[o0];
        }
    }


    // split faces
    for (auto f: mesh.faces())
    {
        if (vselected && !fselected[f])
        {
            continue;
        }

        Surface_mesh::Halfedge h0 = mesh.halfedge(f);

        Surface_mesh::Halfedge h6 = mesh.next_halfedge(mesh.next_halfedge(h0));
        Surface_mesh::Halfedge h1 = mesh.insert_edge(h0, h6);

        Surface_mesh::Halfedge h8 = mesh.next_halfedge(mesh.next_halfedge(h1));
        Surface_mesh::Halfedge h2 = mesh.insert_edge(h1, h8);

        Surface_mesh::Halfedge h3 = mesh.insert_edge(h2, h0);

        Surface_mesh::Halfedge o1 = mesh.opposite_halfedge(h1);
        Surface_mesh::Halfedge o2 = mesh.opposite_halfedge(h2);
        Surface_mesh::Halfedge o3 = mesh.opposite_halfedge(h3);
        Surface_mesh::Halfedge h4 = mesh.next_halfedge(o3);
        Surface_mesh::Halfedge h5 = mesh.next_halfedge(o1);
        Surface_mesh::Halfedge h7 = mesh.next_halfedge(o2);

        if (texcoords)
        {
            texcoords[h0] = etexcoords[h0];
            texcoords[h1] = etexcoords[h6];
            texcoords[h2] = etexcoords[h8];
            texcoords[h3] = etexcoords[h0];
            texcoords[h6] = etexcoords[h6];
            texcoords[h8] = etexcoords[h8];
            texcoords[o1] = etexcoords[h0];
            texcoords[o2] = etexcoords[h6];
            texcoords[o3] = etexcoords[h8];
        }

        if (hnormals)
        {
            Normal normal_1 = hnormals[h0];
            if (normal_1[0] == 0 && normal_1[1] == 0 && normal_1[2] == 0)
            {
                normal_1 = hnormals[h5];
            }
            Normal normal_2 = hnormals[h6];
            if (normal_2[0] == 0 && normal_2[1] == 0 && normal_2[2] == 0)
            {
                normal_2 = hnormals[h7];
            }
            Normal normal_3 = hnormals[h8];
            if (normal_3[0] == 0 && normal_3[1] == 0 && normal_3[2] == 0)
            {
                normal_3 = hnormals[h4];
            }

            hnormals[h0] = (normal_1 + normal_3) * Scalar(0.5);
            hnormals[h1] = (normal_1 + normal_2) * Scalar(0.5);
            hnormals[h2] = (normal_2 + normal_3) * Scalar(0.5);
            hnormals[h3] = (normal_1 + normal_3) * Scalar(0.5);
            hnormals[h4] = normal_3;
            hnormals[h5] = normal_1; 
            hnormals[h6] = (normal_1 + normal_2) * Scalar(0.5);
            hnormals[h7] = normal_2;
            hnormals[h8] = (normal_2 + normal_3) * Scalar(0.5);
            hnormals[o1] = (normal_1 + normal_3) * Scalar(0.5);
            hnormals[o2] = (normal_1 + normal_2) * Scalar(0.5);
            hnormals[o3] = (normal_2 + normal_3) * Scalar(0.5);
        }

        if (vselected && fselected[f])
        {
            Surface_mesh::Vertex v0 = mesh.to_vertex(h0);
            Surface_mesh::Vertex v1 = mesh.to_vertex(h1);
            Surface_mesh::Vertex v2 = mesh.to_vertex(h2);
            vselected[v0] = true;
            vselected[v1] = true;
            vselected[v2] = true;

            Surface_mesh::Face fh0 = mesh.face(h0);
            Surface_mesh::Face fh1 = mesh.face(h1);
            Surface_mesh::Face fh5 = mesh.face(h5);
            Surface_mesh::Face fh7 = mesh.face(h7);
            fselected[fh0] = true;
            fselected[fh1] = true;
            fselected[fh5] = true;
            fselected[fh7] = true;

            Surface_mesh::Halfedge o0  = mesh.opposite_halfedge(h0);
            Surface_mesh::Halfedge o6  = mesh.opposite_halfedge(h6);
            Surface_mesh::Halfedge o8  = mesh.opposite_halfedge(h8);
            Surface_mesh::Face     fo0 = mesh.face(o0);
            Surface_mesh::Face     fo6 = mesh.face(o6);
            Surface_mesh::Face     fo8 = mesh.face(o8);

            if (mesh.is_valid(fo0) && !fselected[fo0])
            {
                Surface_mesh::Halfedge o0n = mesh.next_halfedge(o0);
                Surface_mesh::Halfedge o0p = mesh.prev_halfedge(o0);
                Surface_mesh::Halfedge temp_h0 = mesh.insert_edge(o0p, o0n);

                Surface_mesh::Halfedge temp_h1 = mesh.next_halfedge(temp_h0);
                Surface_mesh::Halfedge temp_o1 = mesh.opposite_halfedge(temp_h1);
                Surface_mesh::Halfedge temp_h2 = mesh.next_halfedge(temp_h1);
                Surface_mesh::Halfedge temp_h3 = mesh.opposite_halfedge(temp_h0);
                Surface_mesh::Halfedge temp_h4 = mesh.next_halfedge(temp_h3);
                Surface_mesh::Halfedge temp_h5 = mesh.next_halfedge(temp_h4);

                if (texcoords)
                {
                    texcoords[temp_h0] = texcoords[temp_o1];
                    texcoords[temp_h1] = texcoords[h5];
                    texcoords[temp_h2] = texcoords[h0];
                    texcoords[temp_h3] = texcoords[h0];
                    texcoords[temp_h4] = texcoords[h4];
                    texcoords[temp_h5] = texcoords[temp_o1];
                }

                if (hnormals)
                {
                    hnormals[temp_h0] = hnormals[temp_o1];
                    hnormals[temp_h1] = hnormals[h5];
                    hnormals[temp_h2] = hnormals[h0];
                    hnormals[temp_h3] = hnormals[h0];
                    hnormals[temp_h4] = hnormals[h4];
                    hnormals[temp_h5] = hnormals[temp_o1];
                }
            }
            if (mesh.is_valid(fo6) && !fselected[fo6])
            {
                Surface_mesh::Halfedge o6n = mesh.next_halfedge(o6);
                Surface_mesh::Halfedge o6p = mesh.prev_halfedge(o6);
                Surface_mesh::Halfedge temp_h0 = mesh.insert_edge(o6p, o6n);

                Surface_mesh::Halfedge temp_h1 = mesh.next_halfedge(temp_h0);
                Surface_mesh::Halfedge temp_o1 = mesh.opposite_halfedge(temp_h1);
                Surface_mesh::Halfedge temp_h2 = mesh.next_halfedge(temp_h1);
                Surface_mesh::Halfedge temp_h3 = mesh.opposite_halfedge(temp_h0);
                Surface_mesh::Halfedge temp_h4 = mesh.next_halfedge(temp_h3);
                Surface_mesh::Halfedge temp_h5 = mesh.next_halfedge(temp_h4);

                if (texcoords)
                {
                    texcoords[temp_h0] = texcoords[temp_o1];
                    texcoords[temp_h1] = texcoords[h7];
                    texcoords[temp_h2] = texcoords[h6];
                    texcoords[temp_h3] = texcoords[h6];
                    texcoords[temp_h4] = texcoords[h5];
                    texcoords[temp_h5] = texcoords[temp_o1];
                }

                if (hnormals)
                {
                    hnormals[temp_h0] = hnormals[temp_o1];
                    hnormals[temp_h1] = hnormals[h5];
                    hnormals[temp_h2] = hnormals[h0];
                    hnormals[temp_h3] = hnormals[h0];
                    hnormals[temp_h4] = hnormals[h4];
                    hnormals[temp_h5] = hnormals[temp_o1];
                }
            }
            if (mesh.is_valid(fo8) && !fselected[fo8])
            {
                Surface_mesh::Halfedge o8n = mesh.next_halfedge(o8);
                Surface_mesh::Halfedge o8p = mesh.prev_halfedge(o8);
                Surface_mesh::Halfedge temp_h0 = mesh.insert_edge(o8p, o8n);

                Surface_mesh::Halfedge temp_h1 = mesh.next_halfedge(temp_h0);
                Surface_mesh::Halfedge temp_o1 = mesh.opposite_halfedge(temp_h1);
                Surface_mesh::Halfedge temp_h2 = mesh.next_halfedge(temp_h1);
                Surface_mesh::Halfedge temp_h3 = mesh.opposite_halfedge(temp_h0);
                Surface_mesh::Halfedge temp_h4 = mesh.next_halfedge(temp_h3);
                Surface_mesh::Halfedge temp_h5 = mesh.next_halfedge(temp_h4);

                if (texcoords)
                {
                    texcoords[temp_h0] = texcoords[temp_o1];
                    texcoords[temp_h1] = texcoords[h4];
                    texcoords[temp_h2] = texcoords[h8];
                    texcoords[temp_h3] = texcoords[h8];
                    texcoords[temp_h4] = texcoords[h7];
                    texcoords[temp_h5] = texcoords[temp_o1];
                }

                if (hnormals)
                {
                    hnormals[temp_h0] = hnormals[temp_o1];
                    hnormals[temp_h1] = hnormals[h5];
                    hnormals[temp_h2] = hnormals[h0];
                    hnormals[temp_h3] = hnormals[h0];
                    hnormals[temp_h4] = hnormals[h4];
                    hnormals[temp_h5] = hnormals[temp_o1];
                }
            }
        }
    }


    // clean-up properties
    mesh.remove_vertex_property(vpoint);
    mesh.remove_edge_property(epoint);
    mesh.remove_halfedge_property(etexcoords);
    mesh.remove_face_property(fselected);

    if (!hnormals)
    {
        mesh.update_face_normals();
        mesh.update_vertex_normals();
    }
}


//=============================================================================
} // namespace surface_mesh
} // namespace graphene
//=============================================================================

