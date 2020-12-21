layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform mat4 modelview_projection_matrix;
uniform float width;
uniform float height;
uniform vec2 position_offset;

out vec2 texcoord;

void main() 
{
    gl_Position = //modelview_projection_matrix *
        vec4( width + position_offset.x, height + position_offset.y, 0.0, 1.0 );
    texcoord = vec2( 1.0, 1.0 );
    EmitVertex();

    gl_Position = //modelview_projection_matrix *
        vec4(-width + position_offset.x, height + position_offset.y, 0.0, 1.0 );
    texcoord = vec2( 0.0, 1.0 ); 
    EmitVertex();

    gl_Position = //modelview_projection_matrix *
        vec4( width + position_offset.x,-height + position_offset.y, 0.0, 1.0 );
    texcoord = vec2( 1.0, 0.0 ); 
    EmitVertex();

    gl_Position = //modelview_projection_matrix *
        vec4(-width + position_offset.x,-height + position_offset.y, 0.0, 1.0 );
    texcoord = vec2( 0.0, 0.0 ); 
    EmitVertex();

    EndPrimitive(); 
}