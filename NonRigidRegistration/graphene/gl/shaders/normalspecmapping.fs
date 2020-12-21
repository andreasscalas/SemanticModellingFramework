in vec3 v2f_normal;
in vec3 v2f_color;
in vec3 v2f_texcoords;
in vec4 v2f_eyepos;

// light sources
uniform vec3 light_directions[8];
uniform vec3 light_colors[8];
uniform int  num_active_lights = 1;

// material
uniform vec3      front_color = vec3(0.5, 0.55, 0.6);
uniform vec3      back_color  = vec3(0.5, 0.0, 0.0);
uniform vec4      material    = vec4(0.1, 0.8, 1.0, 100.0);
uniform sampler2D texture2D;
uniform sampler2D texture2D_nm;
uniform sampler2D texture2D_spec;

// switches
uniform bool use_texture2D   = false;
uniform bool use_vertexcolor = false;
uniform bool use_lighting    = true;

// out
out vec4 out_color;

// tmp
vec3 color_tmp;


//-----------------------------------------------------------------------------


void phong_lighting(const vec3  normal,
                    const vec4  material,
                    const vec3  color,
                    inout vec3  result)
{
    // ambient component
    result = vec3(0.0);
    if (use_texture2D)
    {
        result += material[0] * color * texture(texture2D, v2f_texcoords.xy).xyz;
    }
    else
    {
        result += material[0] * color;
    }

    vec3  L, R;
    vec3 V = normalize(-v2f_eyepos.xyz);
    float NL, RV;

    // for each light source
    for (int i=0; i < num_active_lights; ++i)
    {
        L = light_directions[i];

        // diffuse
        NL = dot(normal, L);
        if (NL > 0.0)
        {
            if (use_texture2D)
            {
                result += NL * material[1] * color * texture(texture2D, v2f_texcoords.xy).xyz * light_colors[i];
            }
            else
            {
                result += NL * material[1] * color * light_colors[i];
            }

            // specular
            R  = normalize(reflect(-L, normal));
            RV = dot(R, V);
            if (RV > 0.0)
            {
                if (use_texture2D)
                {
                    vec3 spec_tmp = texture(texture2D_spec, v2f_texcoords.xy).xyz;
                    result += spec_tmp * material[2] * pow(RV, material[3]);
                }
                else
                {
                    result += vec3(material[2] * pow(RV, material[3]));
                }
            }
        }
    }
}


//-----------------------------------------------------------------------------


mat3 cotangent_frame( vec3 N, vec3 p, vec2 uv )
{
    // get edge vectors of the pixel triangle
    vec3 dp1 = dFdx( p );
    vec3 dp2 = dFdy( p );
    vec2 duv1 = dFdx( uv );
    vec2 duv2 = dFdy( uv );

    // solve the linear system
    vec3 dp2perp = cross( dp2, N );
    vec3 dp1perp = cross( N, dp1 );
    vec3 T = dp2perp * duv1.x + dp1perp * duv2.x;
    vec3 B = dp2perp * duv1.y + dp1perp * duv2.y;

    // construct a scale-invariant frame 
    float invmax = inversesqrt( max( dot(T,T), dot(B,B) ) );

    return mat3( T * invmax, B * invmax, N );
}


//-----------------------------------------------------------------------------


vec3 perturb_normal( vec3 N, vec3 V, vec2 texcoord )
{
    // assume N, the interpolated vertex normal and
    // V, the view vector (vertex to eye)
    vec3 map = texture( texture2D_nm, texcoord ).xyz;
    map = map * 255./127. - 128./127.;


    mat3 TBN = cotangent_frame( N, -V, texcoord );
    return normalize( TBN * map );
}


//-----------------------------------------------------------------------------


void main()
{
    vec3 N = normalize(v2f_normal);


    // see: <http://www.thetenthplanet.de/archives/1180>
    vec3 V = -v2f_eyepos.xyz; // non normalized view vector
    if (use_texture2D)
    {
        N = perturb_normal( N, V, v2f_texcoords.xy );
    }


    if (use_lighting)
    {
        vec3 color = front_color;
        if (use_vertexcolor)
            color = v2f_color;

        if (gl_FrontFacing)
        {
            phong_lighting(N, material, color, color_tmp);
        }
        else
        {
            phong_lighting(-N, material, back_color, color_tmp);
        }
    }
    else
    {
        if (use_vertexcolor)
        {
            color_tmp = v2f_color;
        }
        else
        {
            color_tmp = front_color;
        }
    }

    color_tmp = clamp(color_tmp, 0.0, 1.0);


    out_color = vec4(color_tmp, 1.0);
}
