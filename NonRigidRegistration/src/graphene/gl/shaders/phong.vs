
// in
in vec4 vertex;
in vec3 normal;
in vec4 color;
in vec3 texcoords;

// out
out vec3 v2f_normal;
out vec3 v2f_color;
out vec3 v2f_texcoords;
out vec4 v2f_eyepos;
out vec4 v2f_ShadowCoord0;
out vec4 v2f_ShadowCoord1;

// uniform
uniform mat4 model_matrix;
uniform mat4 modelview_projection_matrix;
uniform mat3 normal_matrix;
uniform mat4 modelview_matrix;

uniform mat4 viewproj0;
uniform mat4 viewproj1;

void main()
{
    v2f_normal    = normalize(normal_matrix * normal);
    v2f_color     = color.xyz;
    v2f_texcoords = texcoords;
    v2f_eyepos    = modelview_matrix * vertex;

    mat4 Bmat = mat4( 0.5, 0.0, 0.0, 0.0,   // 1st column
                      0.0, 0.5, 0.0, 0.0,   // 2nd column
                      0.0, 0.0, 0.5, 0.0,   // 3rd column
                      0.5, 0.5, 0.5, 1.0 ); // 4th column

    v2f_ShadowCoord0 = Bmat * viewproj0 * model_matrix * vec4(vertex.xyz, 1.0);
    v2f_ShadowCoord1 = Bmat * viewproj1 * model_matrix * vec4(vertex.xyz, 1.0);

    gl_Position   = modelview_projection_matrix * vertex;
}
