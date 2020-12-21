
precision highp float;


// in
in vec4  vertex;
in vec4  color;
in float radius;
in vec3  direction;

// out
out mat4 MT_inverse;
out vec4 color_vs;

// uniforms
uniform mat4  mvp_transpose;
uniform mat4  mv_inverse;
uniform vec4  viewport;
uniform float uniform_radius = 0.0;

// constants
const vec4 D = vec4(1,1,0,-1);


void main(void )
{
    float r;
    if (uniform_radius > 0.0)
    {
        r = uniform_radius;
    }
    else
    {
        r = radius;
    }


    // local coord system
    vec3 w = direction.xyz;
    vec3 u = r * normalize(cross(w, w.yzx));
    vec3 v = r * normalize(cross(w, u));
    vec3 c = vertex.xyz;


    vec4 bbox1, bbox2, bbox;
    vec4 r1, r2, r4, Dr1, Dr2, Dr4, eqn;
    mat4 T_transpose, PMT_transpose;




    //--- bounding box for cylinder cap at z=+1 ---

    // move center to end of cylinder
    c.xyz = vertex.xyz + direction.xyz;

    // setup T^T
    T_transpose = mat4(u.x, v.x, w.x, c.x,
                       u.y, v.y, w.y, c.y,
                       u.z, v.z, w.z, c.z,
                       0.0, 0.0, 0.0, 1.0);

    // build (P*M*T)^T = T^T * (P*M)^T
    PMT_transpose = T_transpose * mvp_transpose;

    // rows of (P*M*T) are columns of (P*M*T)^T
    r1 = PMT_transpose[0];
    r2 = PMT_transpose[1];
    r4 = PMT_transpose[3];

    // multiply with diagonal matrix D
    Dr1 = D*r1;
    Dr2 = D*r2;
    Dr4 = D*r4;

    // two quadratic equations for bounding box size in x/y
    eqn.x = dot(r1, Dr4);
    eqn.y = dot(r2, Dr4);
    eqn.z = dot(r1, Dr1);
    eqn.w = dot(r2, Dr2);
    eqn  /= dot(r4, Dr4);
    eqn.zw = sqrt((eqn.xy * eqn.xy) - eqn.zw);
    bbox1.xy = eqn.xy - eqn.zw; // smaller solution
    bbox1.zw = eqn.xy + eqn.zw; // bigger solution


    //--- bounding box for cylinder cap at z=-1 ---

    // move center to end of cylinder
    c.xyz = vertex.xyz - direction.xyz;

    // update rows of P*M*T, which are columns of (P*M*T)^T
    r1.w = dot(c, mvp_transpose[0].xyz) + mvp_transpose[0].w;
    r2.w = dot(c, mvp_transpose[1].xyz) + mvp_transpose[1].w;
    r4.w = dot(c, mvp_transpose[3].xyz) + mvp_transpose[3].w;

    // multiply with diagonal matrix D
    Dr1 = D*r1;
    Dr2 = D*r2;
    Dr4 = D*r4;

    // two quadratic equations for bounding box size in x/y
    eqn.x = dot(r1, Dr4);
    eqn.y = dot(r2, Dr4);
    eqn.z = dot(r1, Dr1);
    eqn.w = dot(r2, Dr2);
    eqn  /= dot(r4, Dr4);
    eqn.zw = sqrt((eqn.xy * eqn.xy) - eqn.zw);
    bbox2.xy = eqn.xy - eqn.zw; // smaller solution
    bbox2.zw = eqn.xy + eqn.zw; // bigger solution

    ///--- bounding boxes of both caps have been computed ---



    // combine two bounding boxes
    bbox.xy = min(bbox1.xy, bbox2.xy); // lower bound for x and y
    bbox.zw = max(bbox1.zw, bbox2.zw); // upper bound for x and y


    // point size
    vec2 bbsize = bbox.zw - bbox.xy;
    bbsize *= viewport.xy;
    gl_PointSize = max(bbsize.x, bbsize.y);


    // transformed position is bbox center
    gl_Position.xy = 0.5 * (bbox.xy + bbox.zw);
    gl_Position.zw = vec2(0.0,1.0);


    // construct T^{-1}
    c.xyz = vertex.xyz;
    u /= dot(u,u);
    v /= dot(v,v);
    w /= dot(w,w);
    mat4 T_inverse = mat4(u.x, v.x, w.x, 0.0,
                          u.y, v.y, w.y, 0.0,
                          u.z, v.z, w.z, 0.0,
                          -dot(u,c), -dot(v,c), -dot(w,c), 1.0);


    // pass (M*T)^{-1} to fragment shader
    MT_inverse = T_inverse * mv_inverse;


    // pass color_vs to fragment shader
    color_vs = color;
}
