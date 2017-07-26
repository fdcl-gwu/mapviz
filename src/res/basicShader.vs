#version 450

attribute vec3 position;
attribute vec2 texCoord;
attribute vec3 normal;
in vec4 color;

varying vec2 texCoord0;
varying vec3 normal0;

uniform mat4 MVP;
uniform mat4 Normal;

out vec4 color0;

void main()
{
	color0 = color;
	gl_Position = MVP * vec4(position, 1.0);
	texCoord0 = texCoord;
	normal0 = (Normal * vec4(normal, 0.0)).xyz;
}
