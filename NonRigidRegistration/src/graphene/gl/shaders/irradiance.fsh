#extension  GL_ARB_sample_shading : enable

in vec3 v2f_normal;
in vec3 v2f_color;
in vec3 v2f_texcoords;
in vec4 v2f_eyepos;

in vec4 v2f_ShadowCoord0;
in vec4 v2f_ShadowCoord1;

uniform float mixRatio = 0.5;

// light sources
uniform vec3 light_directions[8];
uniform vec3 light_colors[8];
uniform int  num_active_lights = 1;
uniform sampler2D albedomap;
uniform sampler2D texture2D_nm;

//uniform sampler2D irradiance_texture;

// switches
uniform bool use_texture2D   = false;

uniform bool use_normalmap;

// shadow mapping stuff
uniform bool use_shadowmaps;
uniform float strength = 0.5;
uniform sampler2DShadow shadowmap0;
uniform sampler2DShadow shadowmap1;


out vec4 out_color;


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


void main()
{

  vec3 N = normalize(v2f_normal);

  vec3 V = -v2f_eyepos.xyz; // non normalized view vector
  if (use_texture2D && use_normalmap)
  {
    N = perturb_normal( N, V, v2f_texcoords.xy );
  }

  vec3 diffuseIrradiance = vec3(0.0);

  for (int i=0; i<num_active_lights; ++i)
  {
    vec3 L = light_directions[i];
    float ndotl = dot(N, L);
    diffuseIrradiance += max(ndotl, 0.0) * light_colors[i];
  }
  diffuseIrradiance *= pow( texture(albedomap, v2f_texcoords.xy).xyz, vec3( mixRatio ) );


  // do the shadow-map look-up
  float shadow = 0.0;

  if (use_shadowmaps)
  {
    shadow += shadowPCF(v2f_ShadowCoord0, shadowmap0);
    shadow += shadowPCF(v2f_ShadowCoord1, shadowmap1);
  }

  shadow /= 50.0f;
  shadow *= strength;

  out_color = vec4( (1.0f - shadow) * diffuseIrradiance.xyz, 1.0 );
}
