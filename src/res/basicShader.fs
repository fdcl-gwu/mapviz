#version 450

in vec4 color0;
varying vec2 texCoord0;
in vec3 normal0;

uniform sampler2D sampler;
uniform vec3 lightDirection;

void main()
{

	gl_FragColor = texture2D(sampler, texCoord0) * clamp(dot(-lightDirection, normal0), 0.0, 1.0);

	gl_FragColor = color0;
	if (gl_FragColor.a < 0.5)
      // alpha value less than user-specified threshold?
  {
      discard; // yes: discard this fragment
  }

}
