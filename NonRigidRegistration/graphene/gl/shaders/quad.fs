out vec4 color;

in vec2 texcoord;

uniform sampler2D myTextureSampler;

void main() 
{
//	vec4 tex = texel2D(texture, texcoord);
	//color = vec4(0.0, texcoord.x, texcoord.y, 0.5);
	color = texture(myTextureSampler, texcoord).rgba;
	//gl_FragDepth = -1.5;
}