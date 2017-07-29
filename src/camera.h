#ifndef CAMERA_INCLUDED_H
#define CAMERA_INCLUDED_H

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

struct Camera
{
public:
	Camera(const glm::vec3& pos, float fov, float aspect, float zNear, float zFar)
	{
		this->position = pos;
		this->forward = glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f));
		this->up = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
		this->projection = glm::perspective(fov, aspect, zNear, zFar);
		this->rotation = glm::lookAt(position, position + forward, up);
	}

	inline glm::mat4 GetViewProjection() const
	{
		return projection * rotation;//glm::lookAt(position, position + forward, up);
	}

	inline void setForward(const glm::vec3& forward)
	{
		this->forward = forward;
	}
	inline void setUp(const glm::vec3& up)
	{
		this->up = up;
	}
	inline void setPos(const glm::vec3& pos)
	{
		this->position = pos;
	}
	inline glm::vec3 getPos()
	{
		return position;
	}
	inline void setRot(const glm::mat4& rotation)
	{
		// this->forward = glm::normalize(rotation * glm::vec3(0.0f, 0.0f, 1.0f));
		// this->up = glm::normalize(rotation * glm::vec3(0.0f, 1.0f, 0.0f));
		this->rotation = rotation;

	}
	inline void setRot(const glm::mat3& rotation)
	{
		// this->forward = glm::normalize(rotation * glm::vec3(0.0f, 0.0f, 1.0f));
		// this->up = glm::normalize(rotation * glm::vec3(0.0f, 1.0f, 0.0f));
		this->rotation = glm::lookAt(position, position + forward, up);
	}

	//void MoveForward(float amt)
	//{
	//	pos += forward * amt;
	//}

	//void MoveRight(float amt)
	//{
	//	pos += glm::cross(up, forward) * amt;
	//}

	//void Pitch(float angle)
	//{
	//	glm::vec3 right = glm::normalize(glm::cross(up, forward));

	//	forward = glm::vec3(glm::normalize(glm::rotate(angle, right) * glm::vec4(forward, 0.0)));
	//	up = glm::normalize(glm::cross(forward, right));
	//}

	//void RotateY(float angle)
	//{
	//	static const glm::vec3 UP(0.0f, 1.0f, 0.0f);

	//	glm::mat4 rotation = glm::rotate(angle, UP);

	//	forward = glm::vec3(glm::normalize(rotation * glm::vec4(forward, 0.0)));
	//	up = glm::vec3(glm::normalize(rotation * glm::vec4(up, 0.0)));
	//}

protected:
private:
	glm::mat4 projection;
	glm::vec3 position;
	glm::vec3 forward;
	glm::vec3 up;
	glm::mat4 rotation;
};

#endif
