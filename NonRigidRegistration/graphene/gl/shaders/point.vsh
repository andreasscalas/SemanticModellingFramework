
// in
in vec4 vertex;
in vec3 normal;
in vec4 color;

// out
out vec3 v2f_normal;
out vec3 v2f_color;
out vec4 v2f_eyepos;

// uniform
uniform mat4 modelview_projection_matrix;
uniform mat3 normal_matrix;
uniform mat4 modelview_matrix;
uniform vec4 nhtb;
uniform float radius;


void main()
{
    v2f_normal    = normalize(normal_matrix * normal);
    v2f_color     = color.xyz;
    vec4 ep       = modelview_matrix * vertex;
    v2f_eyepos    = ep;

    gl_Position   = modelview_projection_matrix * vertex;
    
    gl_PointSize  = radius * ( ( nhtb.x/-ep.z ) * ( nhtb.y/(nhtb.z - nhtb.w) ) );
    
}
