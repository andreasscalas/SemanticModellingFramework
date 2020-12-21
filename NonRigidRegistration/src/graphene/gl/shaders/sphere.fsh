#extension  GL_ARB_sample_shading : enable

// in
in vec4 color_vs;
in mat4 MT_inverse;

// out
out vec4 out_color;

// uniforms
uniform vec3  light_directions[8];
uniform vec3  light_colors[8];
uniform int   num_active_lights = 1;
uniform vec4  material = vec4(0.1, 0.8, 1.0, 100.0);
uniform vec2  near_far;     // (n, f/(f-n))
uniform mat4  VP_inverse;

const vec4  D = vec4(1,1,1,-1);

vec3  N,L,R, color_tmp;
float a, b, c, d;


//-----------------------------------------------------------------------------


void phong_lighting(const vec3  normal,
                    const vec4  material,
                    const vec3  color,
                    inout vec3  result)
{
    // ambient component
    result = material[0] * color;

    vec3  L, R;
    const vec3 V = vec3(0.0, 0.0, -1.0);
    float NL, RV;

    // for each light source
    for (int i=0; i < num_active_lights; ++i)
    {
        L = light_directions[i];

        // diffuse
        NL = dot(normal, L);
        if (NL > 0.0)
        {
            result += NL * material[1] * color * light_colors[i];

            // specular
            R  = normalize(reflect(L, normal));
            RV = dot(R, V);
            if (RV > 0.0)
            {
                result += vec3(material[2] * pow(RV, material[3]));
            }
        }
    }

    result = clamp(result, 0.0, 1.0);
}


//-----------------------------------------------------------------------------



void main(void)
{
    // pixel in window coordinates (on near plane)
    vec4 x_w = vec4(gl_FragCoord.x + gl_SamplePosition.x,
                    gl_FragCoord.y + gl_SamplePosition.y,
                    0.0,
                    1.0);

    // convert to eye coordinates by inverting viewport and projection mapping
    vec4 x_e = VP_inverse*x_w;
    x_e /= x_e.w;
    x_e.z = -near_far.x;

    
    // construct ray r(lambda) = o + v*lambda in parameter space
    // direction is (x_e,0), origin is (0,0,0,1), which are mapped to parameter space
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


    // x in parameter space, equals normal in parameter space
    vec4 x_p = o + lambda*v;


    // normal in eye space: (M*T)^{-T} * x_p
    N.x = dot(x_p.xyz, MT_inverse[0].xyz);
    N.y = dot(x_p.xyz, MT_inverse[1].xyz);
    N.z = dot(x_p.xyz, MT_inverse[2].xyz);
    N = normalize(N);


    // lighting
    phong_lighting(N, material, color_vs.xyz, color_tmp);

    out_color = vec4(color_tmp, 1.0);
}
