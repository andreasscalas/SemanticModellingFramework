// in
in vec4 vertex;
in vec3 texcoords;
in vec3 normal;
in vec4 color;

// out
out vec3 v2f_texcoords;
out vec3 v2f_normal;
out vec3 v2f_color;
out vec3 v2f_worldcoords;
out vec4 v2f_eyepos;
out vec4 v2f_ShadowCoord0;
out vec4 v2f_ShadowCoord1;

// uniform
uniform mat4 model_matrix;
uniform mat3 normal_matrix;
uniform mat4 modelview_matrix;

uniform mat4 viewproj0;
uniform mat4 viewproj1;

void main ()
{
    //convert texture coordinates to NDC
    vec3 pos = texcoords * 2 - 1;

    v2f_normal = normalize(normal_matrix * normal);
    v2f_color = color.xyz;
    v2f_texcoords = texcoords;
    v2f_eyepos    = modelview_matrix * vertex;
    v2f_worldcoords = vec3(model_matrix * vertex);

    mat4 Bmat = mat4( 0.5, 0.0, 0.0, 0.0,   // 1st column
                      0.0, 0.5, 0.0, 0.0,   // 2nd column
                      0.0, 0.0, 0.5, 0.0,   // 3rd column
                      0.5, 0.5, 0.5, 1.0 ); // 4th column

    v2f_ShadowCoord0 = Bmat * viewproj0 * model_matrix * vec4(vertex.xyz, 1.0);
    v2f_ShadowCoord1 = Bmat * viewproj1 * model_matrix * vec4(vertex.xyz, 1.0);

    gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
}
