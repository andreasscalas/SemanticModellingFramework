in vec3 v2f_normal;
in vec3 v2f_color;
in vec3 v2f_texcoords;
in vec4 v2f_eyepos;

in vec4 v2f_ShadowCoord0;
in vec4 v2f_ShadowCoord1;

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

// blurred irradiance textures
uniform sampler2D irrad1Tex;
uniform sampler2D irrad2Tex;
uniform sampler2D irrad3Tex;
uniform sampler2D irrad4Tex;
uniform sampler2D irrad5Tex;
uniform sampler2D irrad6Tex;

// RGB Gaussian weights that define skin profiles
uniform vec3 gauss1w;
uniform vec3 gauss2w;
uniform vec3 gauss3w;
uniform vec3 gauss4w;
uniform vec3 gauss5w;
uniform vec3 gauss6w;

// mix ratio
uniform float mixRatio = 0.5;

// switches
uniform bool use_texture2D   = false;
uniform bool use_vertexcolor = false;
uniform bool use_lighting    = true;


uniform bool use_normalmap;
uniform bool use_specularmap;


// shadow mapping stuff
uniform bool use_shadowmaps;
uniform float strength = 0.5;
uniform sampler2DShadow shadowmap0;
uniform sampler2DShadow shadowmap1;


// out
out vec4 out_color;

// tmp
vec3 color_tmp;


//-----------------------------------------------------------------------------


float shadowPCF(vec4 shadowCoord, sampler2DShadow shadowMap)
{
  float shadow = 0.0;

  const float stepsize = 1.0 / 1024.0;

  if (shadowCoord.w > 0.0)
  {
    // sample an 5x5 values from the shadow map

    for (float x = -2.0f; x <= 2.001; x += 1.0f)
    {
        for (float y = -2.0f; y <= 2.001; y += 1.0f)
        {
            vec4 offset_coord = vec4(shadowCoord.xy + vec2(x*stepsize,y*stepsize), shadowCoord.zw);
            shadow += textureProj( shadowMap, offset_coord);
        }
    }
  }

  return shadow;
}


//-----------------------------------------------------------------------------


void phong_lighting(const vec3  normal,
                    const vec4  material,
                    const vec3  color,
                    inout vec3  result)
{

    // TODO RAUS ?!

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
                if(use_texture2D && use_specularmap)
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


float computeFresnelReflectance(vec3 H, vec3 V, float F0)
{
    float base = 1.0 - dot(V,H);
    float exponential = pow(base, 5.0);
    return exponential + F0 * (1.0 - exponential);
}


//-----------------------------------------------------------------------------


// instead of light distribution by cos^n(R,V) as assumed in the Phong-Model,
// use the Beckmann-Distribution model.
float PHBeckmann(float ndoth, float m)
{
    float alpha = acos(ndoth);
    float ta = tan(alpha);

    return 1.0/(m*m*pow(ndoth, 4.0)) * exp(-(ta*ta)/(m*m));
}


//-----------------------------------------------------------------------------


void skin_shader_lighting(const vec3  normal,
                          const vec4  material,
                          const vec3  color,
                          inout vec3  result)
{
    // ambient component
    result = vec3(0.0);
    result += material[0] * color;

    // compute the diffuse lighting, i.e., sum of the weighted six irradiance textues
    vec3 finalDiffuseColor = vec3(0.0);
    vec3 irrad1tap = texture( irrad1Tex, v2f_texcoords.xy ).xyz;
    finalDiffuseColor += irrad1tap * gauss1w;
    vec3 irrad2tap = texture( irrad2Tex, v2f_texcoords.xy ).xyz;
    finalDiffuseColor += irrad2tap * gauss2w;
    vec3 irrad3tap = texture( irrad3Tex, v2f_texcoords.xy ).xyz;
    finalDiffuseColor += irrad3tap * gauss3w;
    vec3 irrad4tap = texture( irrad4Tex, v2f_texcoords.xy ).xyz;
    finalDiffuseColor += irrad4tap * gauss4w;
    vec3 irrad5tap = texture( irrad5Tex, v2f_texcoords.xy ).xyz;
    finalDiffuseColor += irrad5tap * gauss5w;
    vec3 irrad6tap = texture( irrad6Tex, v2f_texcoords.xy ).xyz;
    finalDiffuseColor += irrad6tap * gauss6w;

    // renormalize diffusion profile to white
    vec3 normConst = gauss1w + gauss2w + gauss3w + gauss4w + gauss5w + gauss6w;   // TODO VERMUTLICH UNWICHTIG ?!
    finalDiffuseColor /= normConst;

    // compute diffuse albedo color from the albedo texture
    vec3 diffuseAlbedo = texture( texture2D, v2f_texcoords.xy ).xyz;
    finalDiffuseColor *= material[1] * pow( diffuseAlbedo, vec3( 1.0 - mixRatio ) );


    // specular lighting
    // for each light source
    vec3 specularLight = vec3( 0.0 );

    vec3  L, R;
    vec3 V = normalize(-v2f_eyepos.xyz);
    float NL, RV;

    // constant for fresnel-term, optimal for skin
    float F0 = 0.028;
    // constant roughness-value for skin
    float m = 0.12;

    // physically based
    for(int i=0; i < num_active_lights; i++)
    {
        L = light_directions[i];
        float NL = dot(normal, L);
        if ( NL > 0.0 )
        {
            vec3 h = L + V;
            vec3 H = normalize(h);
            float NH = dot(normal, H);
            float PH = PHBeckmann(NH, m);
            float F = computeFresnelReflectance(H, V, F0);
            float frSpec = max(PH * F / dot(h,h), 0);
            if(use_texture2D && use_specularmap)
            {
                vec3 spec_tmp = texture(texture2D_spec, v2f_texcoords.xy).xyz;
                specularLight += light_colors[i] * spec_tmp * NL * material[2] * frSpec; // rho_s ist aus spec-map und material[2] // shadow siehe unten
            }
            else
            {
                specularLight += light_colors[i] * NL * material[2] * frSpec; // rho_s ist aus spec-map und material[2] // shadow siehe unten
            }
        }
    }


    // do the shadow-map look-up
    float shadow = 0.0;

    if (use_shadowmaps)
    {
      shadow += shadowPCF(v2f_ShadowCoord0, shadowmap0);
      shadow += shadowPCF(v2f_ShadowCoord1, shadowmap1);
    }

    shadow /= 50.0f;
    shadow *= strength;


    result += finalDiffuseColor;
    result += ( (1.0f - shadow) * specularLight );
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
    if (use_texture2D && use_normalmap)
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
            skin_shader_lighting(N, material, color, color_tmp);
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
