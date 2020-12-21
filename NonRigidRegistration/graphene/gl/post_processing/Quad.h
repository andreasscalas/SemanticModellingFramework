#ifndef GRAPHENE_QUAD_GL_H
#define GRAPHENE_QUAD_GL_H

#include "../gl_includes.h"
#include <graphene/geometry/Matrix4x4.h>

namespace graphene {
namespace gl {

struct Quad {
private:
    GLuint m_va;
    GLuint m_vb;
    GLuint m_tcb;
    Vec3f m_verts[6];
    float m_tex_coords[12];

public:

    /*!
      \brief Constructor

      Construct a quad from triangle with the specified width.
      \param[in] width the width of the quad
    */
    Quad(float width = 1.0f): m_va(0), m_vb(0), m_tcb(0)
    {
        m_verts[0][0] = 0.0f;    m_verts[0][1] = 0.0f;         m_verts[0][2] = 0.0f;
        m_verts[1][0] = width;   m_verts[1][1] = 0.0f;         m_verts[1][2] = 0.0f;
        m_verts[2][0] = width;   m_verts[2][1] = width;         m_verts[2][2] = 0.0f;

        m_verts[3][0] = 0.0f;         m_verts[3][1] = 0.0f;    m_verts[3][2] = 0.0f;
        m_verts[4][0] = width;        m_verts[4][1] = width;   m_verts[4][2] = 0.0f;
        m_verts[5][0] = 0.0f;         m_verts[5][1] = width;   m_verts[5][2] = 0.0f;

        m_tex_coords[0] = 0.0f; m_tex_coords[1] = 0.0f;
        m_tex_coords[2] = 1.0f; m_tex_coords[3] = 0.0f;
        m_tex_coords[4] = 1.0f; m_tex_coords[5] = 1.0f;

        m_tex_coords[6] = 0.0f; m_tex_coords[7] = 0.0f;
        m_tex_coords[8] = 1.0f; m_tex_coords[9] = 1.0f;
        m_tex_coords[10] = 0.0f; m_tex_coords[11] = 1.0f;
    }

    /// destructor
    ~Quad()
    {
        //m_verts.clear();
        if (m_vb) glDeleteBuffers(1, &m_vb);
        if (m_tcb) glDeleteBuffers(1, &m_tcb);
        if (m_va) glDeleteVertexArrays(1, &m_va);
    }

    /// inits VBOs and VAO
    void init()
    {

        glGenVertexArrays(1, &m_va);
        glBindVertexArray(m_va);

        glGenBuffers(1, &m_vb);
        glBindBuffer(GL_ARRAY_BUFFER, m_vb);
        glBufferData(GL_ARRAY_BUFFER, 18*sizeof(float), &m_verts, GL_STATIC_DRAW );
        glVertexAttribPointer(attrib_locations::VERTEX, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(attrib_locations::VERTEX);

        glGenBuffers(1, &m_tcb);
        glBindBuffer(GL_ARRAY_BUFFER, m_tcb);
        glBufferData(GL_ARRAY_BUFFER, 12*sizeof(float), &m_tex_coords, GL_STATIC_DRAW);
        glVertexAttribPointer(attrib_locations::TEXCOORDS, 2, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(attrib_locations::TEXCOORDS);

        glBindVertexArray(0);

    }

    /// draws the quad
    void draw()
    {
        glBindVertexArray(m_va);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
    }
};

} // namespace gl
} // namespace graphene

#endif //GRAPHENE_QUAD_GL_H
