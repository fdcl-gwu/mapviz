#ifndef CUBE_H
#define CUBE_H
#include <iostream>
#include <algorithm>
#include "SDL2/SDL.h"
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include "mesh.h"

using namespace std;
using namespace glm;



vector<vec3> getPosition(vec3 pos, float voxel_size)
{
	float half_size = voxel_size;
	vector<vec3> translate;
	translate.push_back(vec3( pos.x, pos.y, half_size + pos.z));
	translate.push_back(vec3( pos.x, half_size + pos.y, half_size + pos.z));
	translate.push_back(vec3(half_size + pos.x, half_size + pos.y, half_size + pos.z));
	translate.push_back(vec3(half_size + pos.x, pos.y, half_size + pos.z));
	translate.push_back(vec3( pos.x,  pos.y, pos.z));
	translate.push_back(vec3( pos.x, half_size + pos.y, pos.z));
	translate.push_back(vec3(half_size + pos.x, half_size + pos.y,  pos.z));
	translate.push_back(vec3(half_size + pos.x,  pos.y,  pos.z));
	return translate;
};

vector<vec3> normal =
{
	vec3(0.0, 0.0, 1.0),
	vec3(1.0, 0.0, 0.0),
	vec3(0.0, 1.0, 0.0),
	vec3(0.0, 1.0, 0.0),
	vec3(0.0, 0.0, 1.0),
	vec3(1.0, 0.0, 0.0)
};

std::vector<Vertex> cube_vertex;
std::vector<unsigned int> cube_index;

void quad( int a, int b, int c, int d, int face, vector<vec3> positions, vec2 texture, vec4 color)
{
	cube_index.push_back(a);cube_index.push_back(b);cube_index.push_back(c);
	cube_index.push_back(a);cube_index.push_back(c);cube_index.push_back(d);

	cube_vertex.push_back(Vertex(positions[a], texture, normal[face], color));
	cube_vertex.push_back(Vertex(positions[b], texture, normal[face], color));
	cube_vertex.push_back(Vertex(positions[c], texture, normal[face], color));

	cube_vertex.push_back(Vertex(positions[a], texture, normal[face], color));
	cube_vertex.push_back(Vertex(positions[c], texture, normal[face], color));
	cube_vertex.push_back(Vertex(positions[d], texture, normal[face], color));
};

void colorcube(vec3 min, vec3 range, float grid_size){
	vec3 N(floor(range.x/grid_size + 0.5),
			floor(range.y/grid_size + 0.5),
			floor(range.z/grid_size + 0.5));
	cout<<"Dimentions cube List: "<<N.x<<"x"<<N.y<<"x"<<N.z<<endl;
	cube_vertex.reserve(N.x * N.y * N.z);
	vec2 texture;
	vec4 color(0.0f, 0.0f, 0.0f, 0.0f);
	// float scale = grid_size;
	vector<vec3> pos = getPosition(vec3(0,0,0), 0.2);
	for(int k = 0; k < N.z; ++k){
		for(int j = 0; j < N.y; ++j){
			for(int i = 0; i < N.x; ++i){
			pos = getPosition(vec3(range.x * (float)i/N.x + min.x,
				range.y * (float)j/N.y + min.y, range.z * (float)k/N.z + min.z), grid_size*0.9);
			color.x = (float)j/N.y;
			color.y = (float)k/N.x;
			quad(1,0,3,2,0, pos, texture, color); // front
			quad(2,3,7,6,1, pos, texture, color); // right
			quad(3,0,4,7,2, pos, texture, color); // bottom
			quad(6,5,1,2,3, pos, texture, color); // top
			quad(4,5,6,7,4, pos, texture, color); // back
			quad(5,4,0,1,5, pos, texture, color); // left
			}
		}
	}

};


#endif
