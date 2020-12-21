
// in
in vec3  vertex;
in vec4  color;
in float radius;

// out
out mat4 MT_inverse;
out vec4 color_vs;

// uniforms
uniform mat4  mvp_transpose;
uniform mat4  mv_inverse;
uniform vec2  width_height;

// constants
const vec4 D = vec4(1,1,1,-1);


void main (void)
{
    // setup T^T and T^{-1}
    mat4 T_transpose = mat4(radius, 0.0, 0.0, vertex.x,
                            0.0, radius, 0.0, vertex.y,
                            0.0, 0.0, radius, vertex.z,
                            0.0, 0.0, 0.0, 1.0);
    float ir = 1.0/radius;
    mat4 T_inverse   = mat4(ir, 0.0, 0.0, 0.0,
                            0.0, ir, 0.0, 0.0,
                            0.0, 0.0, ir, 0.0,
                            -ir*vertex.x, -ir*vertex.y, -ir*vertex.z, 1.0);


    // build (P*M*T)^T = T^T * (P*M)^T
    mat4 PMT_transpose = T_transpose * mvp_transpose;


    // rows of (P*M*T) are columns of (P*M*T)^T
    vec4 r1 = PMT_transpose[0];
    vec4 r2 = PMT_transpose[1];
    vec4 r4 = PMT_transpose[3];


    // multiply with diagonal matrix D
    vec4 Dr1 = D*r1;
    vec4 Dr2 = D*r2;
    vec4 Dr4 = D*r4;


    // two quadratic equations for bounding box size in x/y
    // compute difference between two solutions, some terms cancel out
    vec4 eqn;
    eqn.x = dot(r1, Dr4);
    eqn.y = dot(r2, Dr4);
    eqn.z = dot(r1, Dr1);
    eqn.w = dot(r2, Dr2);
    eqn  /= dot(r4, Dr4);
    vec2 bbsize = ((eqn.xy * eqn.xy) - eqn.zw); // sqrt(bbsize) is radius in clip coords
    if (bbsize.x < 0.0 || bbsize.y < 0.0)
    {
        gl_PointSize = 1.0;
    }
    else
    {
        bbsize = sqrt(bbsize) * width_height; // diameter in window coords
        gl_PointSize = max(bbsize.x, bbsize.y);
    }


    // vertex position
    gl_Position = vec4(eqn.xy, 0.0, 1.0);


    // pass (M*T)^{-1} to fragment shader
    MT_inverse = T_inverse * mv_inverse;


    // pass color_vs to fragment shader
    color_vs = color;
}
