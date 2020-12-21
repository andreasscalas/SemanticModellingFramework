//in
in vec4 vertex;
in vec3 normal;
in vec3 color;
in vec3 texcoords;
in vec4 weights;
in vec4 depends;
in vec4 weights2;
in vec4 depends2;

// out
out vec3 v2f_normal;
out vec3 v2f_color;
out vec3 v2f_texcoords;
out vec4 v2f_eyepos;
out vec4 v2f_ShadowCoord0;
out vec4 v2f_ShadowCoord1;

// uniform
uniform mat4 model_matrix;
uniform mat4 modelview_projection_matrix;
uniform mat3 normal_matrix;
uniform mat4 modelview_matrix;

uniform mat4 viewproj0;
uniform mat4 viewproj1;


layout(std140) uniform skinning_matrices {
    mat4 matrices[256]; 
} skin_matrices;


//temps
vec4 vertexSkin;
vec4 normalSkin;

mat4 accMat;

void main()
{

    v2f_color = color;
    v2f_texcoords = texcoords;
    
    
    
    //blend matrices
    accMat  = weights[0] * skin_matrices.matrices[ int(depends[0]) ];
    accMat += weights[1] * skin_matrices.matrices[ int(depends[1]) ];
    accMat += weights[2] * skin_matrices.matrices[ int(depends[2]) ];
    accMat += weights[3] * skin_matrices.matrices[ int(depends[3]) ];
    
    accMat += weights2[0] * skin_matrices.matrices[ int(depends2[0]) ];
    accMat += weights2[1] * skin_matrices.matrices[ int(depends2[1]) ];
    accMat += weights2[2] * skin_matrices.matrices[ int(depends2[2]) ];
    accMat += weights2[3] * skin_matrices.matrices[ int(depends2[3]) ];

    //transform vertex
    vertexSkin = accMat * vertex;
    normalSkin = accMat * vec4(normal, 0);

    v2f_normal = normalize(normal_matrix * normalSkin.xyz);
    v2f_eyepos    = modelview_matrix * vertexSkin;

    mat4 Bmat = mat4( 0.5, 0.0, 0.0, 0.0,   // 1st column
                      0.0, 0.5, 0.0, 0.0,   // 2nd column
                      0.0, 0.0, 0.5, 0.0,   // 3rd column
                      0.5, 0.5, 0.5, 1.0 ); // 4th column

    v2f_ShadowCoord0 = Bmat * viewproj0 * model_matrix * vec4(vertexSkin.xyz, 1.0);
    v2f_ShadowCoord1 = Bmat * viewproj1 * model_matrix * vec4(vertexSkin.xyz, 1.0);

    gl_Position = modelview_projection_matrix * vertexSkin;
} 
