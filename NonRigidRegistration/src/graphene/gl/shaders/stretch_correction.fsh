
in vec3 v2f_worldcoords;
in vec3 v2f_texcoords;

uniform float scale;

out vec2 out_color;

void main() {

  vec3 derivu = dFdx( v2f_worldcoords );
  vec3 derivv = dFdy( v2f_worldcoords );
  float stretchU = scale / length( derivu );
  float stretchV = scale / length( derivv );

  out_color = vec2( stretchU, stretchV );
//  out_color = vec2( 1.0, 1.0 );
}
