
precision highp float;


// in
in vec4 color_vs;
in mat4 MT_inverse;

// out
out vec4 out_color;

// uniforms
uniform vec3  light_directions[8];
uniform vec3  light_colors[8];
uniform int   num_active_lights = 1;
uniform vec4  viewport;
uniform mat4  p_inverse;
uniform vec4  front_ambient_material;
uniform vec4  front_diffuse_material;
uniform vec4  front_specular_material;
uniform float front_specular_shininess;
uniform bool  use_lighting = true;

const vec4 D = vec4(1,1,0,-1);

// tmp
vec4 x_c, x_e, x_p, c3, eqn;
vec3 N,L,R, color_tmp;


//-----------------------------------------------------------------------------


void phong_lighting(const vec3  normal,
                    const vec3  ambient,
                    const vec3  diffuse,
                    const vec3  specular,
                    const float shininess,
                    inout vec3  result)
{
    // ambient component (assume light_ambient=(1,1,1))
    result = ambient;

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
            result += NL * diffuse * light_colors[i];

            // specular
            R  = normalize(reflect(L, normal));
            RV = dot(R, V);
            if (RV > 0.0)
            {
                result += specular * pow(RV, shininess);
            }
        }
    }

    result = clamp(result, 0.0, 1.0);
}


//-----------------------------------------------------------------------------


void main(void)
{
    // x' in clip space
    x_c.xy = (gl_FragCoord.xy * viewport.zw) - 1.0;
    x_c.zw = vec2(0.0, 1.0);


    // x' in parameter space
    x_p = MT_inverse * (p_inverse * x_c);


    // c3: third column of (P*M*T)^{-1}
    c3 = MT_inverse * p_inverse[2];


    // setup quadratic equation
    eqn.x = dot(c3,  D*c3);
    eqn.y = dot(c3,  D*x_p);
    eqn.z = dot(x_p, D*x_p);
    eqn.yz /= eqn.x;
    eqn.w = (eqn.y * eqn.y) - eqn.z;


    // discriminant < 0 => kill
    if (eqn.w < 0)
    {
        discard;
    }


    // solve quadratic equation
    x_c.z = -sqrt(eqn.w) - eqn.y;
    if (x_c.z < -1.0)
    {
        discard;
    }


    // clip-z to window-z: [-1,1] -> [0,1]
    gl_FragDepth = x_c.z*0.5 + 0.5;


    // x in parameter space, equals normal in parameter space
    x_p.xyzw += c3.xyzw * x_c.z;


    // outside length => kill
    if (abs(x_p.z/x_p.w) > 1.0)
    {
        discard;
    }

    // normal in eye space: (M*T)^{-T} * x_p  (but x_p.z should be 0)
    N.x = dot(x_p.xy, MT_inverse[0].xy);
    N.y = dot(x_p.xy, MT_inverse[1].xy);
    N.z = dot(x_p.xy, MT_inverse[2].xy);
    N = normalize(N);


    if (use_lighting)
    {
        // x in eye space: add to x_e' third column of p_inverse times z
        //x_e = p_inverse * x_c;
        //x_e.xyz /= x_e.w; // can save this since we will normalize x_e anyway

        phong_lighting(N,
                       front_ambient_material.xyz,
                       front_diffuse_material.xyz * color_vs.xyz,
                       front_specular_material.xyz,
                       front_specular_shininess,
                       color_tmp);
    }
    else
    {
        color_tmp = color_vs.xyz;
    }

    out_color = vec4(color_tmp, 1.0);
}
