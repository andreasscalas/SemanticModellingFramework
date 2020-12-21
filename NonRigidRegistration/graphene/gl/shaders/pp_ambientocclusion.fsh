//Code adapted from "Horizon Based Ambient Occlusion".

#define M_PI 3.1415926535898
#define TEX_SIZE 63

in vec2 v2f_texcoords;

uniform vec2 resolution;
uniform vec2 inv_resolution;

uniform float near;
uniform float far;

uniform float Radius = 0.01;
uniform float max_steps = 8.0;
uniform float number_directions = 8.0;
uniform float Attenuation = 0.0;
uniform float AngleBias = 0.0;
uniform float Contrast = 1.15;


uniform sampler2D depth_texture;
uniform sampler2D effect_texture;
uniform sampler2D randomtex;


float num_steps;
vec3 random;
float ao, d, alpha;
vec2 step_size;
vec3 P, N, R;
vec3 Pr, Pl, Pt, Pb, dPdu, dPdv;
vec3 color_vec;
vec4 tangentPlane;

out vec4 out_color;


float length2(vec3 v) { return dot(v, v); }

vec3 fetch_eye_pos(vec2 uv)
{
    //get previously linearized z
    float z = texture(depth_texture, uv).x;
    return vec3(((uv) ) * z, z);
}



vec3 min_diff(vec3 P, vec3 Pr, vec3 Pl)
{
    vec3 V1 = Pr - P;
    vec3 V2 = P - Pl;
    return (length2(V1) < length2(V2)) ? V1 : V2;
}

vec2 rotate_direction(vec2 Dir, vec2 CosSin)
{
    return vec2(Dir.x*CosSin.x - Dir.y*CosSin.y,
                  Dir.x*CosSin.y + Dir.y*CosSin.x);
}

float falloff(float r)
{
    return 1.0 - Attenuation*r;
}

vec2 snap_uv_offset(vec2 uv)
{
    return round(uv * resolution)/resolution;
}




float tangent(vec3 T)
{
    return -T.z * (1.0/ length(T.xy));
}
float tangent(vec3 P, vec3 S)
{
    return (P.z - S.z) / length(S.xy - P.xy);
}

float biased_tangent(vec3 T)
{
    float phi = atan(tangent(T)) + AngleBias;
    return tan(min(phi, M_PI*0.5));
}
float accumulate(vec2 deltaUV,
                                  vec2 uv0,
                                  vec3 P,
                                  float numSteps,
                                  float randstep,
                                  vec3 dPdu,
                                  vec3 dPdv )
{
    // Randomize starting point within the first sample distance
    vec2 uv = uv0 + snap_uv_offset( randstep * deltaUV );

    // Snap increments to pixels to avoid disparities between xy
    // and z sample locations and sample along a line
    deltaUV = snap_uv_offset( deltaUV );

    // Compute tangent vector using the tangent plane
    vec3 T = deltaUV.x * dPdu + deltaUV.y * dPdv;

    float tanH = biased_tangent(T);
    float sinH = tanH / sqrt(1.0f + tanH*tanH);

    float ao = 0;
    for(float j = 1; j <= numSteps; ++j) {
        uv += deltaUV;
        vec3 S = fetch_eye_pos(uv);

        T = S - P;
        // Ignore any samples outside the radius of influence
        float d2  = length(T);
        if (d2 < Radius) {
            float tanS = tangent(T);

            if(tanS > tanH) {
                // Accumulate AO between the horizon and the sample
                float sinS = tanS / sqrt(1.0f + tanS*tanS);
                float r = (d2) * (1.0/Radius);
                ao += falloff(r) * (sinS - sinH);

                // Update the current horizon angle
                tanH = tanS;
                sinH = sinS;
            }
        }
    }

    return ao;
}


vec3 getRandom() {
    return texture(randomtex, v2f_texcoords * 64).xyz;
}


void main () {

    P = fetch_eye_pos(v2f_texcoords);

    if (P.z >= 0.999) {
        discard;
    }

    step_size = vec2(0.5 * Radius  / P.z);

    num_steps = min(max_steps, min(step_size.x * resolution.x, step_size.y * resolution.y));



    if (num_steps <= 1.0) {
        out_color = vec4(1.0,1.0,1.0, 1.0);
    }
    else {

        step_size = step_size/(num_steps + 1);


        Pr = fetch_eye_pos(v2f_texcoords + vec2( inv_resolution.x, 0.0));
        Pl = fetch_eye_pos(v2f_texcoords + vec2(-inv_resolution.x, 0.0));
        Pt = fetch_eye_pos(v2f_texcoords + vec2(0.0,  inv_resolution.y));
        Pb = fetch_eye_pos(v2f_texcoords + vec2(0.0, -inv_resolution.y));


        dPdu = min_diff(P, Pr, Pl);
        dPdv = min_diff(P, Pt, Pb) * (resolution.y/resolution.x);

        ao = 0.0;
        alpha = (2.0 * M_PI) / number_directions;

        random = getRandom();//vec3(1.0);

        for (d = 0; d < number_directions; ++d) {
            float angle = alpha * d;
            vec2 deltaUV = rotate_direction(vec2(cos(angle), sin(angle)), random.xy) * step_size;
            ao += accumulate(deltaUV, v2f_texcoords, P, num_steps, (random.z), dPdu, dPdv);
        }

        out_color =  vec4(vec3(1.0 - ao/number_directions * Contrast),1.0) * texture(effect_texture, v2f_texcoords);
    }

    //gl_FragDepth = P.z;
}