#version 330 core

in vec3 position;
in vec2 texCoord;
in vec3 normal;
in vec4 color;

uniform mat4 MVP;
uniform mat4 Normal;

out vec4 color0;
out vec3 normal0;
out vec2 texCoord0;
void main()
{
 	color0 = color;
  normal0 = (Normal * vec4(normal, 0.0)).xyz;
	gl_Position = MVP * vec4(position, 1.0);
	texCoord0 = texCoord;
}
