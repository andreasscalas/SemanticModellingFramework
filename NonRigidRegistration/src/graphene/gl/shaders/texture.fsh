#extension  GL_ARB_sample_shading : enable

in vec2 v2f_texcoords;

uniform sampler2D texture_of_interest;

out vec4 out_color;

void main() {

    out_color = vec4(
                  texture(texture_of_interest, v2f_texcoords).xyz
                ,1.0);

}
