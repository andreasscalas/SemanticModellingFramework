
in vec2 v2f_texcoords;

//uniform float num_samples = 1.5;
//uniform float subtrahend;
uniform float strength = 0.7;
//uniform float penumbra_size = 0.002;
uniform float shadowmap_size = 1024.0;

uniform sampler2DShadow shadowmap0;
uniform sampler2DShadow shadowmap1;
uniform sampler2DMS depth_texture;
uniform sampler2D effect_texture;

uniform mat4 viewproj0;
uniform mat4 viewproj1;
uniform mat4 viewproj_inv;

out vec4 out_color;

void main() {

    vec2 uv,offset;
    vec4 v;
    float shadow = 0.0;
    float stepsize = 1.0/shadowmap_size;
    float depth;

    depth = texelFetch(depth_texture, ivec2(gl_FragCoord.xy), 0).x;

    //build vertex in normalized device coords
    vec4 vertex = vec4(v2f_texcoords, depth, 1.0);
    vertex.xyz = vertex.xyz * 2.0 - 1.0;

    //transform vertex in camera coords
    vertex = (viewproj_inv * vertex);


    offset = vec2(
                float(fract(gl_FragCoord.x*0.5) > 0.25),
                float(fract(gl_FragCoord.y*0.5) > 0.25)
                );
    offset.y += offset.x;

    if (offset.y > 1.1)
        offset.y = 0.0;

    v = viewproj0 * vertex;

    if (v.w >= 0.0)
    {
        //devide by w and go from [-1,1] to [0,1]
        v.xyz = (v.xyz / v.w) * 0.5 + 0.5;
        //offset
        v.z -= 0.005;

        uv = v.xy + (offset+vec2(-1.5, 0.5)) * stepsize;
        shadow += texture(shadowmap0, vec3(uv,v.z));

        uv = v.xy + (offset+vec2( 0.5, 0.5)) * stepsize;
        shadow += texture(shadowmap0, vec3(uv,v.z));

        uv = v.xy + (offset+vec2(-1.5,-1.5)) * stepsize;
        shadow += texture(shadowmap0, vec3(uv,v.z));

        uv = v.xy + (offset+vec2( 0.5,-1.5)) * stepsize;
        shadow += texture(shadowmap0, vec3(uv,v.z));
    }

    v = viewproj1 * vertex;

    if (v.w >= 0.0)
    {
        //devide by w and go from [-1,1] to [0,1]
        v.xyz = (v.xyz / v.w) * 0.5 + 0.5;
        //offset
        v.z -= 0.005;

        uv = v.xy + (offset+vec2(-1.5, 0.5)) * stepsize;
        shadow += texture(shadowmap1, vec3(uv,v.z));

        uv = v.xy + (offset+vec2( 0.5, 0.5)) * stepsize;
        shadow += texture(shadowmap1, vec3(uv,v.z));

        uv = v.xy + (offset+vec2(-1.5,-1.5)) * stepsize;
        shadow += texture(shadowmap1, vec3(uv,v.z));

        uv = v.xy + (offset+vec2( 0.5,-1.5)) * stepsize;
        shadow += texture(shadowmap1, vec3(uv,v.z));
    }

    shadow *= 0.125*strength;

    out_color = vec4(1.0-shadow) * texture(effect_texture, v2f_texcoords);
}
