in vec4 vertex;
in vec2 texcoords;

uniform float width;
uniform float height;

out vec2 v2f_texcoords;

void main ()
{
    v2f_texcoords = texcoords;
    gl_Position   = vec4(width  * (2.0*vertex.x-1.0),
                         height * (2.0*vertex.y-1.0),
                         0.0,
                         1.0);
}
