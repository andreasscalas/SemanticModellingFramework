
in vec2 v2f_texcoords;

uniform sampler2D texture_to_blur; // Texture being convolved

uniform sampler2D stretch_map; // Stretch correction texture

uniform vec2 inv_resolution;
uniform float factor;
uniform float scale;
uniform float GaussWidth; // Scale - Used to widen Gaussian taps. GaussWidth should be the standard deviation.
uniform float Weight[7] = float[](0.006, 0.061, 0.242, 0.383, 0.242, 0.061, 0.006);
uniform int direction = 0; //0 = horizontal blur, 1 = vertical blur

vec3 sum;

out vec4 out_color;

vec3 blurHorizontal() {

    float scaleConv = inv_resolution.x;
    vec2 stretch = texture( stretch_map, v2f_texcoords ).xy;
    float netFilterWidth = scaleConv * factor * GaussWidth * stretch.x / scale;
    vec2 coords = v2f_texcoords - vec2( netFilterWidth * 3.0, 0.0 );
    vec4 sum = vec4(0.0);
    for ( int i = 0; i < 7; i++ )
    {
        vec4 tap = texture( texture_to_blur, coords );
        sum += Weight[i] * tap;
        coords += vec2( netFilterWidth, 0.0 );
    }

    return sum.xyz;
}

vec3 blurVertical() {

    float scaleConv = inv_resolution.y;
    vec2 stretch = texture( stretch_map, v2f_texcoords ).xy;
    float netFilterWidth = scaleConv * factor * GaussWidth * stretch.y / scale;
    vec2 coords = v2f_texcoords - vec2( 0.0, netFilterWidth * 3.0 );
    vec4 sum = vec4(0.0);
    for ( int i = 0; i < 7; i++ )
    {
        vec4 tap = texture( texture_to_blur, coords );
        sum += Weight[i] * tap;
        coords += vec2( 0.0, netFilterWidth );
    }

    return sum.xyz;
}

void main() {

    switch (direction)
    {
        case 0:
            sum = blurHorizontal();
            break;
        case 1:
            sum = blurVertical();
            break;
    }

    out_color = vec4(sum, 1.0);
}