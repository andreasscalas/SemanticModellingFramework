in vec4 vertex;
in vec2 texcoords;

const mat4 ortho_matrix = mat4(2,0,0,0,
                               0,2,0,0,
                               0,0,-1,0,
                              -1,-1,0,1);

out vec2 v2f_texcoords;

void main () 
{
    v2f_texcoords = texcoords;
    gl_Position = ortho_matrix * vertex;
}
 
