out vec4 color;

in vec2 v2f_texcoords;

uniform sampler2D myTextureSampler;

void main() 
{
    color = texture(myTextureSampler, v2f_texcoords).rgba;
}