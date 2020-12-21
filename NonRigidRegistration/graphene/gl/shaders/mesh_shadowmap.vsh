
// in
in vec4 vertex;


// uniform
uniform mat4 modelview_projection_matrix;


void main()
{
    gl_Position   = modelview_projection_matrix * vertex;
}

