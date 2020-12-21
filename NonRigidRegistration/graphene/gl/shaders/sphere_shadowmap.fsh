
// in
in vec4 color_vs;
in mat4 MT_inverse;

// out
out vec4 out_color;

// uniforms
uniform vec2  near_far;     // (n, f/(f-n))
uniform mat4  VP_inverse;

const vec4  D = vec4(1,1,1,-1);

vec3  N,L,R, color_tmp;
float a, b, c, d;


void main(void)
{
    vec4 x_w = vec4(gl_FragCoord.x, gl_FragCoord.y, 0.0, 1.0);
    vec4 x_e = VP_inverse*x_w;
    x_e /= x_e.w;
    x_e.z = -near_far.x;
    x_e.w = 0.0;
    vec4 v = MT_inverse * x_e;
    vec4 o = MT_inverse[3];


    // solve quadratic equation for window-z
    a = dot(v, D*v);
    b = dot(o, D*v);
    c = dot(o, D*o);
    b /= a;
    c /= a;
    d = b*b - c;
    if (d < 0) discard;
    float lambda = min(-b-sqrt(d), -b+sqrt(d));


    // write window z: convert lambda into z_e and z_e into z_w
    gl_FragDepth = near_far.y - near_far.y/lambda;
}
