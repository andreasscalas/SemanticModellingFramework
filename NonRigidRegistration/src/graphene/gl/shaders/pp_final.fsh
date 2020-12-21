#extension  GL_ARB_sample_shading : enable

in vec2 v2f_texcoords;

uniform sampler2D effect_texture;
uniform sampler2DMS depth_texture;

out vec4 out_color;

void main() {

    float depth = texelFetch(depth_texture, ivec2(gl_FragCoord.xy), gl_SampleID).x;

    if (depth >= 1.0)
    {
        discard;
    }
    gl_FragDepth = depth;

    out_color = vec4(
                  texture(effect_texture, v2f_texcoords).xyz
                ,1.0);

}
