
in vec2 v2f_texcoords;

uniform sampler2D color_texture;
uniform sampler2D effect_texture;

out vec4 out_color;

void main() {
    vec4 color  = texture(color_texture,  v2f_texcoords);
    vec4 effect = texture(effect_texture, v2f_texcoords);

    out_color = vec4(color.rgb * effect.rgb, color.a);
}
