#ifndef MESH_INCLUDED_H
#define MESH_INCLUDED_H

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "obj_loader.h"

struct Vertex
{
public:
	Vertex(const glm::vec3& pos = glm::vec3(0.0f,0.0f,0.0f),
		const glm::vec2& texCoord = glm::vec2(0.0f,0.0f),
		const glm::vec3& normal= glm::vec3(0.0f,0.0f,1.0f),
		const glm::vec4& color = glm::vec4(1.0f,0.0f,0.0f,1.0f)
		)
	{
		this->pos = pos;
		this->texCoord = texCoord;
		this->normal = normal;
		this->color = color;
	}

	glm::vec3* GetPos() { return &pos; }
	glm::vec2* GetTexCoord() { return &texCoord; }
	glm::vec3* GetNormal() { return &normal; }
	glm::vec4* GetColor() { return &color; }

// private:
	glm::vec3 pos;
	glm::vec2 texCoord;
	glm::vec3 normal;
	glm::vec4 color;
};

enum MeshBufferPositions
{
	POSITION_VB,
	TEXCOORD_VB,
	NORMAL_VB,
	COLOR_VB,
	INDEX_VB
};

class Mesh
{
public:
    Mesh(const std::string& fileName);
	Mesh(std::vector<Vertex> vertices, unsigned int numVertices, std::vector<unsigned int> indices, unsigned int numIndices);

	void Draw();
	void Draw_cube();
	void Update_value(std::vector<glm::vec4>& color_RGBA, int N);
	glm::vec4* getColorMem(){return graph;};

	virtual ~Mesh();
protected:
private:
	glm::vec4 *graph;
	static const unsigned int NUM_BUFFERS = 5;
	void operator=(const Mesh& mesh) {}
	Mesh(const Mesh& mesh) {}

    void InitMesh(const IndexedModel& model);

	GLuint m_vertexArrayObject;
	GLuint m_vertexArrayBuffers[NUM_BUFFERS];
	unsigned int m_numIndices;
};

#endif
