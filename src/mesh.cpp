#include "mesh.h"
#include "util.h"
#include "debugTimer.h"
#include <map>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdlib.h>

Mesh::Mesh(const std::string& fileName)
{
    InitMesh(OBJModel(fileName).ToIndexedModel());
}

void Mesh::InitMesh(const IndexedModel& model)
{
    m_numIndices = model.indices.size();

    glGenVertexArrays(1, &m_vertexArrayObject);
	glBindVertexArray(m_vertexArrayObject);

	glGenBuffers(NUM_BUFFERS, m_vertexArrayBuffers);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[POSITION_VB]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(model.positions[0]) * model.positions.size(), &model.positions[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[TEXCOORD_VB]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.texCoords[0]) * model.texCoords.size(), &model.texCoords[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[NORMAL_VB]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.normals[0]) * model.normals.size(), &model.normals[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[COLOR_VB]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(model.color[0]) * model.color.size(), &model.color[0], GL_WRITE_ONLY);
    glBufferData(GL_ARRAY_BUFFER, sizeof(model.color[0]) * model.color.size(), NULL, GL_STREAM_DRAW);
    graph = (glm::vec4 *)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(int i = 0; i < model.color.size(); i++) {
      graph[i].x = 0;
      graph[i].y = 0;
      graph[i].y = 1.0;
      graph[i].w = 1.0;
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertexArrayBuffers[INDEX_VB]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(model.indices[0]) * model.indices.size(), &model.indices[0], GL_STATIC_DRAW);

	glBindVertexArray(0);
}

Mesh::Mesh(std::vector<Vertex> vertices, unsigned int numVertices, std::vector<unsigned int> indices, unsigned int numIndices)
{
    IndexedModel model;

	for(unsigned int i = 0; i < numVertices; i++)
	{
		model.positions.push_back(*vertices[i].GetPos());
		model.texCoords.push_back(*vertices[i].GetTexCoord());
		model.normals.push_back(*vertices[i].GetNormal());
        model.color.push_back(*vertices[i].GetColor());
	}

	for(unsigned int i = 0; i < numIndices; i++)
        model.indices.push_back(indices[i]);

    InitMesh(model);
}

Mesh::~Mesh()
{
	glDeleteBuffers(NUM_BUFFERS, m_vertexArrayBuffers);
	glDeleteVertexArrays(1, &m_vertexArrayObject);
}

void Mesh::Draw()
{
	glBindVertexArray(m_vertexArrayObject);

	//glDrawElements(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0);
	// glDrawElementsBaseVertex(GL_LINES, m_numIndices, GL_UNSIGNED_INT, 0, 0);
    glDrawElementsBaseVertex(GL_LINES, m_numIndices, GL_UNSIGNED_INT, 0, 0);

	glBindVertexArray(0);
}



void Mesh::Update_value(std::vector<glm::vec4>& color_RGBA, int N){
    // glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[COLOR_VB]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(color_RGBA[0]) * color_RGBA.size(), NULL, GL_DYNAMIC_DRAW);
    // graph = (glm::vec4 *)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for(int i = 0; i < N; i++) {
      graph[i].x = color_RGBA[i].x;
      graph[i].y = color_RGBA[i].y;
      graph[i].z = color_RGBA[i].z;
      graph[i].w = color_RGBA[i].w;
    }
    // glUnmapBuffer(GL_ARRAY_BUFFER);
    // glEnableVertexAttribArray(3);
    // glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 0, 0);
}

void Mesh::Draw_cube()
{
	glBindVertexArray(m_vertexArrayObject);
    // glBindBuffer(GL_ARRAY_BUFFER, m_vertexArrayBuffers[COLOR_VB]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(color_RGBA[0]) * color_RGBA.size(), NULL, GL_DYNAMIC_DRAW);
    // glm::vec4 *graph = (glm::vec4 *)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    // glUnmapBuffer(GL_ARRAY_BUFFER);

    // glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(color_RGBA[0]) * color_RGBA.size(), &color_RGBA[0]);
	//glDrawElements(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0);
	// glDrawElementsBaseVertex(GL_LINES, m_numIndices, GL_UNSIGNED_INT, 0, 0);
    // glDrawElements(GL_TRIANGLES, m_numIndices, GL_UNSIGNED_INT, 0);

    glDrawArrays(GL_TRIANGLES, 0, m_numIndices);

	glBindVertexArray(0);
}
