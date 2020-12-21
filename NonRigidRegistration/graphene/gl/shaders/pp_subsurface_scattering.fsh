//in
in vec2 v2f_texcoords;

//uniforms
uniform sampler2D depth_texture;
uniform sampler2D effect_texture;

uniform float ssslevel = 10.0;
uniform float correction = 50.0;
uniform float standard_dev;
uniform vec3 rgb_weights;

uniform vec2 inv_resolution;
uniform int direction = 0; //0 = horizontal blur, 1 = vertical blur

uniform float gauss_w[6];
uniform float gauss_o[6];


//temps
float center_depth;
float sample_depth;
vec4 center_color ;
vec4 sample_color;
vec4 sum;


//out
out vec4 out_color;
out vec4 out_color_alt;

vec4 blurHorizontal() {
    vec4 result = vec4(0.382 * center_color.rgb,center_color.a);
    vec2 final_width = vec2((ssslevel * standard_dev * inv_resolution.x) / center_depth, 0.0);


    vec2 uv;
    for (int i=0; i < 6; ++i)
    {
        uv = v2f_texcoords + gauss_o[i] * final_width;
        sample_color = texture(effect_texture, uv);
        sample_depth = texture(depth_texture, uv).x;

        sample_color.rgb = mix(
                                sample_color.rgb,
                                center_color.rgb,
                                min(correction * abs(center_depth - sample_depth), 1.0)
                              );
        sample_color.rgb = mix(
                                center_color.rgb,
                                sample_color.rgb,
                                sample_color.a
                              );

        result.rgb += gauss_w[i] * sample_color.rgb;
    }


    return result;
}

vec4 blurVertical() {
    vec4 result = vec4(0.382 * center_color.rgb,center_color.a);
    vec2 final_width = vec2(0.0, (ssslevel * standard_dev * inv_resolution.y) / center_depth);


    vec2 uv;
    for (int i=0; i < 6; ++i)
    {
        uv = v2f_texcoords + gauss_o[i] * final_width;
        sample_color = texture(effect_texture, uv);
        sample_depth = texture(depth_texture, uv).x;

        sample_color.rgb = mix(
                                sample_color.rgb,
                                center_color.rgb,
                                min(correction * abs(center_depth - sample_depth), 1.0)
                              );
        sample_color.rgb = mix(
                                center_color.rgb,
                                sample_color.rgb,
                                sample_color.a
                              );

        result.rgb += gauss_w[i] * sample_color.rgb;
    }


    return result;
}

void main() {


    center_depth = texture(depth_texture, v2f_texcoords).x;
    if (center_depth >= 0.999) {
        discard;
    }
    center_color = texture(effect_texture, v2f_texcoords);

    if ( ((0.5 * (standard_dev*ssslevel)) < center_depth ) || center_color.a < 1.0)
    {
        sum = center_color;
        out_color_alt = vec4(sum.rgb * rgb_weights, center_color.a);
    }
    else
    {
        switch (direction)
        {
            case 0:
                sum = blurHorizontal();
                break;
            case 1:
                sum = blurVertical();
                out_color_alt = vec4(sum.rgb * rgb_weights, 1.0);
                break;
        }
    }

    out_color = sum;

    //gl_FragDepth = center_depth;
}