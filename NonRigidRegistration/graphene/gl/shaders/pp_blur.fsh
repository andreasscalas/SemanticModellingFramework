
in vec2 v2f_texcoords;

uniform sampler2D depth_texture;
uniform sampler2D effect_texture;

uniform float radius = 5;
uniform float falloff = 0.0;
uniform float depth_awareness = 50000.0;
uniform vec2 inv_resolution;
uniform int direction = 0; //0 = horizontal blur, 1 = vertical blur

float center_depth;
float diff;
vec3 sum;
float adaptive_radius;

out vec4 out_color;

vec3 BlurFunction(vec2 uv, float r, inout float w_total)
{
    vec3 c = texture(effect_texture, uv).xyz;
    float d = texture(depth_texture, uv).x;

    float ddiff = d - center_depth;
    float w = exp(-r*r*falloff - ddiff*ddiff*depth_awareness);
    w_total += w;

    return w*c;
}

vec3 blurHorizontal() {
    vec2 uv;
    vec3 b = vec3(0.0);
    float w_total = 0;
    for (float r = -adaptive_radius; r <= adaptive_radius; ++r)
    {
        uv = v2f_texcoords + vec2(r*inv_resolution.x, 0);
        b += BlurFunction(uv, r, w_total);
    }
    return b/w_total;
}

vec3 blurVertical() {
    vec2 uv;
    vec3 b = vec3(0.0);
    float w_total = 0;
    for (float r = -adaptive_radius; r <= adaptive_radius; ++r)
    {
        uv = v2f_texcoords + vec2(0, r*inv_resolution.y);
        b += BlurFunction(uv, r, w_total);
    }
    return b/w_total;
}

void main() {
    center_depth = texture(depth_texture, v2f_texcoords).x;

    adaptive_radius = radius * (1.25 - center_depth);

    switch (direction)
    {
        case 0:
            sum = blurHorizontal();
            break;
        case 1:
            sum = blurVertical();
            break;
    }

    out_color = vec4(sum,1.0);
    //gl_FragDepth = center_depth;
}