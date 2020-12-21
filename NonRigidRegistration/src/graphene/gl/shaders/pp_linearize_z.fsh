
in vec2 v2f_texcoords;

uniform float near;
uniform float far;

uniform sampler2DMS depth_texture;

void main() 
{
    float depth = texelFetch(depth_texture, ivec2(gl_FragCoord.xy), 0).x;
    
    gl_FragDepth = (2 * near) / (far + near - depth * (far - near));
}