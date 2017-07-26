#version 450 core

in vec2 texCoord0;
in vec3 normal0;
in vec4 color0;

uniform sampler2D sampler;
uniform vec3 lightDirection;
uniform float probCutoff;
float cutoff = 0.5;

out vec4 gl_FragColor;
void main()
{
	gl_FragColor = texture2D(sampler, texCoord0) * clamp(dot(-lightDirection, normal0), 0.0, 1.0);
	gl_FragColor = color0 * clamp(dot(-lightDirection, normal0), 0.0, 1.0) +
	 color0 * clamp(dot(lightDirection, normal0), 0.0, 1.0) +
	 color0 * clamp(dot(vec3(-1,0,0), normal0), 0.0, 1.0) +
	 color0 * clamp(dot(vec3(1,0,0), normal0), 0.0, 1.0) +
	 color0 * clamp(dot(vec3(0,1,0), normal0), 0.0, 1.0) +
	 color0 * clamp(dot(vec3(0,-1,0), normal0), 0.0, 1.0);
	 if (gl_FragColor.a < probCutoff)
      // alpha value less than user-specified threshold?
    {
      discard; // yes: discard this fragment
    }
}
