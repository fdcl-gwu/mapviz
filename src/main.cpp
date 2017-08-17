#include <iostream>
#include <algorithm>
#include "SDL2/SDL.h"
#include "display.h"
#include "mesh.h"
#include "shader.h"
#include "texture.h"
#include "transform.h"
#include "camera.h"
#include "voxel_traversal.h"
#include <memory>
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "keyboard_mouse.h"
#include "cube.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/package.h>

#include <Eigen/Dense>
#include <glm/gtc/quaternion.hpp>

static const int DISPLAY_WIDTH = 1600;
static const int DISPLAY_HEIGHT = 900;

using namespace std;
using namespace glm;

template <typename T>
class sub_base
{
  public:
	T msg;
	bool draw = false;
	int N_x, N_y, N_z;
	inline bool getDrawFlag() { return draw; };
	std::vector<Eigen::Vector3i> vao_data;
	void callback(const typename T::ConstPtr &msg_sub)
	{
		cout << msg.layout.dim[0].size << endl;
		msg = *msg_sub;
		N_x = msg.layout.dim[0].size;
		N_y = msg.layout.dim[1].size;
		N_z = msg.layout.dim[2].size;
		draw = true;
	};
};

class sub_map : public sub_base<std_msgs::Float64MultiArray>
{
};

bool map_flag = false, mapRGB_flag = false;
int N_x, N_y, N_z, N_total;
std::vector<double> map_data;
std::vector<vec3> mapRGB_data;

void mapCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	// ROS_INFO("I heard: [%d]", msg->layout.dim[0].size);
	if (!map_flag)
	{
		N_x = msg->layout.dim[0].size;
		N_y = msg->layout.dim[1].size;
		N_z = msg->layout.dim[2].size;
		int Ntotal = N_x * N_y * N_z;
		cout << "Dimentions: " << N_x << "x" << N_y << "x" << N_z << endl;
		map_data.reserve(Ntotal);
	}
	map_flag = true;
	for (int i = 0; i < N_total; ++i)
	{
		map_data[i] = msg->data[i];
	}
}

void mapRGBCallback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
	// ROS_INFO("I heard: [%d]", msg->layout.dim[0].size);
	int Ntotal = 0;
	if (!mapRGB_flag)
	{
		int Nx = msg->layout.dim[0].size;
		int Ny = msg->layout.dim[1].size;
		int Nz = msg->layout.dim[2].size;
		Ntotal = Nx * Ny * Nz;
		cout << "RGB map dimentions: " << Nx << "x" << Ny << "x" << Nz << endl;
		mapRGB_data.reserve(Ntotal);
	}
	mapRGB_flag = true;
	for (int i = 0; i < N_total; ++i)
	{
		mapRGB_data[i].x = (float)std::max(0, (int)msg->data[i * 3]) / 255;
		mapRGB_data[i].y = (float)std::max(0, (int)msg->data[i * 3 + 1]) / 255;
		mapRGB_data[i].z = (float)std::max(0, (int)msg->data[i * 3 + 2]) / 255;
	}
}

bool pcl_flag = false;
std::vector<Vertex> pcl_vertex;
std::vector<unsigned int> pcl_index;
tf::StampedTransform tf_uav;
int pcl_size = 320 * 240 * 150;
// pcl_vertex.reserve(320*240*100);
// pcl_index.reserve(320*240*100);
int index_accum = 0;
void pclCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
	pcl_flag = true;
	// cout<<input->width<<" x "<<input->height<<" point_step "<<input->point_step<<endl;

	sensor_msgs::PointCloud2 cloud_msg;
	sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
	modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
								  "y", 1, sensor_msgs::PointField::FLOAT32,
								  "z", 1, sensor_msgs::PointField::FLOAT32);
	sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "x");
	sensor_msgs::PointCloud2 msg_world;
	try
	{
		// 	tfPCLListener.lookupTransform("/Maya","/world",ros::Time(0), transform);
		pcl_ros::transformPointCloud("/world", tf_uav.inverse(), *input, msg_world);
	}
	catch (tf::TransformException &ex)
	{ // warn if there's an issue (typically looping bag files)
		ROS_WARN("%s\n", ex.what());
		// // 	return;
	}

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input, pcl_pc2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
	// pcl_vertex.clear();
	// pcl_index.clear();
	BOOST_FOREACH (const pcl::PointXYZRGB &pt, temp_cloud->points)
	{
		if (!isnan(pt.x) && index_accum < pcl_size && pt.x < 5 && pt.x > -5 && pt.y < 5 && pt.y > -5) // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		{
			pcl_vertex[index_accum] = Vertex(vec3(pt.x, pt.y, pt.z),
											 vec4((float)pt.r / 255, (float)pt.g / 255, (float)pt.b / 255, 1.0));
			index_accum++;
		}
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "visualizer");
	ros::NodeHandle nh;
	ros::Subscriber mapProbSub, mapRGBSub, pclSub;
	sub_map mapProbListener;

	// sub_base<std_msgs::Int16MultiArray> mapRGBListener;
	// mapRGBSub = nh.subscribe
	//  ("/map_rgb",   1, &sub_base<std_msgs::Int16MultiArray>::callback, &mapRGBListener);
	mapProbSub = nh.subscribe("/map_probabilities", 10, mapCallback);
	mapRGBSub = nh.subscribe("/map_rgb", 10, mapRGBCallback);
	pclSub = nh.subscribe<sensor_msgs::PointCloud2>("/depth_pcl", 10, pclCallback);

	cout << "waiting for map msgs...." << endl;
	cout << "size of mapRGB: " << mapRGB_data.size() << endl;

	for (int k = 0; k < pcl_size; ++k)
	{
		pcl_vertex.push_back(Vertex(vec3(0, 0, 0),
									vec4(0, 0, 0, 0.0)));
		pcl_index.push_back(k);
	}

	tf::TransformListener tfPCLListener;

	float grid_size = 0.075;
	// unsigned int N_x, N_y,N_z;
	float L_x, L_y, L_z;
	float x_min, y_min, z_min;

	Display display(DISPLAY_WIDTH, DISPLAY_HEIGHT, "OpenGL");
	std::vector<Vertex> vert_vec;
	std::vector<unsigned int> idx_vec;
	// while(!pcl_flag)
	// {
	// 		ros::spinOnce();
	// }

	// construct simple test case
	bool test_flag = false;
	if (argc == 2)
	{
		test_flag = true;
		cout << "test value: " << argv[1] << endl;
		grid_size = 0.5;
		L_x = 10, L_y = 10, L_z = 3;
		N_x = L_x / grid_size;
		N_y = L_y / grid_size;
		N_z = L_z / grid_size;
		x_min = -L_x / 2, y_min = -L_y / 2, z_min = 0;
		N_total = N_z * N_y * N_x;
	}
	else
	{
		while (!map_flag || !mapRGB_flag)
		{
			ros::spinOnce();
		}
		cout << "size of mapRGB: " << mapRGB_data.size() << endl;
		cout << "size of map: " << map_data.size() << endl;
		std::cout << "Have " << argc << " arguments:" << std::endl;
		for (int i = 1; i < argc; ++i)
		{
			std::cout << atof(argv[i]) << ", ";
		}
		ros::param::get("z_map_min", z_min);
		L_x = N_x * grid_size;
		L_y = N_y * grid_size;
		L_z = N_z * grid_size;
		N_total = N_z * N_y * N_x;
		x_min = -L_x / 2, y_min = -L_y / 2;
	}
	if (argc == 8)
	{
		x_min = atof(argv[1]), y_min = atof(argv[2]), z_min = atof(argv[3]);
		grid_size = atof(argv[4]);
		L_x = atof(argv[5]), L_y = atof(argv[6]), L_z = atof(argv[7]);
	}

	cout << "Voxel total count: " << N_total << endl;
	float x_max = x_min + L_x;
	float y_max = y_min + L_y;
	float z_max = z_min + L_z;

	vec3 N_dim(N_x, N_y, N_z);

	Vertex vertex(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	vertex.pos.x = x_max;
	vertex.pos.y = y_max;
	vertex.pos.z = z_max;
	vert_vec.push_back(vertex);
	vertex.pos.x = x_min;
	vert_vec.push_back(vertex);
	vert_vec.push_back(vertex);
	vertex.pos.y = y_min;
	vert_vec.push_back(vertex);
	vert_vec.push_back(vertex);
	vertex.pos.x = x_max;
	vert_vec.push_back(vertex);
	vert_vec.push_back(vertex);
	vertex.pos.y = y_max;
	vert_vec.push_back(vertex);

	vertex.pos.z = 0.0; //z_min;
	for (int i = 0; i < N_dim.x + 1; ++i)
	{
		vertex.pos.y = y_min;
		vertex.pos.x = L_x * (float)i / N_dim.x + x_min;
		vert_vec.push_back(vertex);
		vertex.pos.y = L_y + y_min;
		vert_vec.push_back(vertex);

		vertex.pos.x = x_min;
		vertex.pos.y = L_y * (float)i / N_dim.y + y_min;
		vert_vec.push_back(vertex);
		vertex.pos.x = L_x + x_min;
		vert_vec.push_back(vertex);
	}

	for (unsigned int i = 0; i < vert_vec.size(); i += 2)
	{
		auto idx = i;
		idx_vec.push_back(idx);
		idx_vec.push_back(idx + 1);
	}

	colorcube(vec3(x_min, y_min, z_min), vec3(L_x, L_y, L_z), grid_size);

	vector<Vertex> axis_v;
	vector<unsigned int> axis_idx;
	Vertex vert_axr(glm::vec3(0.0f, 0.0f, 0.f), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
	Vertex vert_axg(glm::vec3(0.0f, 0.0f, 0.f), glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
	Vertex vert_axb(glm::vec3(0.0f, 0.0f, 0.f), glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

	axis_v.push_back(vert_axr);
	vert_axr.pos.x += 0.5;
	axis_v.push_back(vert_axr);
	axis_v.push_back(vert_axg);
	vert_axg.pos.y += 0.5;
	axis_v.push_back(vert_axg);
	axis_v.push_back(vert_axb);
	vert_axb.pos.z += 0.5;
	axis_v.push_back(vert_axb);
	for (unsigned int k = 0; k < 6; ++k)
		axis_idx.push_back(k);

	Eigen::Vector3d pos_min;
	pos_min << x_min, y_min, z_min;
	auto ray_cast = Ray_base(x_min, y_min, z_min, L_x, L_y, L_z, grid_size);

	Eigen::Vector3d ray_start(0, 0, 0);
	Eigen::Vector3d ray_end(4, 4, 4);
	ray_cast.setStartEnd(ray_start, ray_end);
	// cout<<"ending ray: \n"<<ray_end<<endl;
	//
	cout << "ray voxel simple: " << ray_cast.SingleRayCasting3D() << endl;

	Mesh mesh(vert_vec, vert_vec.size(), idx_vec, idx_vec.size(), false);
	Mesh cube(cube_vertex, cube_vertex.size(), cube_index, cube_index.size(), false);
	Mesh pcl_mesh(pcl_vertex, pcl_vertex.size(), pcl_index, pcl_index.size(), true);
	Mesh axis(axis_v, axis_v.size(), axis_idx, axis_idx.size(), true);

	auto axis_colorMem = axis.getColorMem();
	auto axis_posMem = axis.getPosMem();
	for (int i = 0; i < axis_v.size(); ++i)
	{
		axis_colorMem[i] = axis_v[i].color;
		axis_posMem[i] = axis_v[i].pos;
	}

	//Mesh monkey("./res/monkey3.obj");
	// cout<<ros::package::getPath("map_viz")<<endl;
	std::string path = ros::package::getPath("mapviz");
	Shader shader(path + "/src/res/basicShader");
	Shader cube_shader(path + "/src/res/cubeShader");
	Texture texture(path + "/src/res/bricks.jpg");
	Transform transform;
	Camera camera(glm::vec3(0.0f, 0.0f, -15.0f), 70.0f,
				  (float)DISPLAY_WIDTH / (float)DISPLAY_HEIGHT, 0.1f, 1000.0f);
	// vector<vec4> color_RGBA;
	// color_RGBA.reserve(cube_vertex.size());
	// int N_v = cube_vertex.size();
	// for(unsigned int i = 0; i < cube_vertex.size(); ++i)
	// {
	// 	color_RGBA.push_back(vec4((float)i/N_v,1.0 - (float)i/N_v,1.0,1.0));
	// }

	cout << cube_vertex.size() << '\n';
	SDL_Event event;
	key_mouse_state km_state;

	double idx_loop = 0;
	int idx_update = 0;
	unsigned int lastTime = 0, currentTime;
	unsigned int FPS_counter = 0;

	while (km_state.isRunning && nh.ok())
	{
		ros::spinOnce();
		SDL_event_handle(event, km_state); // handles key and mouse input
		// glm::mat4 tf_rot;
		// glm::quat tf_quat;
		// glm::vec3 euler;

		try
		{
			tfPCLListener.lookupTransform("/camera_rgb_optical_frame", "/world", ros::Time(0), tf_uav);
			// cout<<tf_uav.getOrigin()[0]<<endl;
			// auto tf_quatv = tf_uav.getRotation();
			// cout<<tf_quatv[0]<<endl;
			// tf_quat = glm::quat(tf_quatv[0], tf_quatv[1],tf_quatv[2],tf_quatv[3]);
			// euler = glm::eulerAngles(tf_quat) * 3.14159f / 180.f;

			// tf_rot = glm::gtc::quaternion::toMat4(tf_quat);
			// cout<<tf_quat.eulerAngles()[0]<<endl;
			// pcl_ros::transformPointCloud("/world", transform.inverse(), *input, msg_world);
		}
		catch (tf::TransformException &ex)
		{   // warn if there's an issue (typically looping bag files)
			// ROS_WARN("%s\n", ex.what());
			// ros::Duration(0.1).sleep();
		}

		// camera.setPos(vec3(tf_uav.getOrigin().x(), tf_uav.getOrigin().y(),tf_uav.getOrigin().z()));
		auto tf_quat = tf_uav.getRotation();
		// cout<<tf_quat[0] <<" "<<tf_quat[1] << " "<<tf_quat[2]<< " " <<tf_quat[3] <<endl;
		auto rotation = quat(tf_quat[0], tf_quat[1], tf_quat[2], tf_quat[3]);
		auto camera_pos = vec3(tf_uav.getOrigin().x(), tf_uav.getOrigin().y(), tf_uav.getOrigin().z());
		// auto camera_pos = vec3(0, 1, 2);
		// Conversion from Euler angles (in radians) to Quaternion
		vec3 EulerAngles(radians(90.f), radians(-90.f), radians(180.f));
		auto ros2glm = quat(EulerAngles);
		auto rotMatrix = glm::mat3_cast(ros2glm * rotation);
		auto eulerRot = glm::eulerAngles(ros2glm * rotation);
		// cout << "euler:  " << eulerRot.x << " " << eulerRot.y << " " << eulerRot.z << endl;

		vec3 CameraAngles(radians(180.f), radians(90.f), radians(90.f));
		auto w2camera = quat(CameraAngles);
		auto rotMatrix2 = glm::mat3_cast(w2camera * rotation);
		auto rotMat4 = glm::mat4_cast(ros2glm * rotation);

		camera_pos = rotMatrix * (-camera_pos);

		// camera.setPos(vec3(camera_pos.x, camera_pos.y, -camera_pos.z));
		// camera.setForward(glm::normalize(rotMatrix2 * vec3(0,1,0)) - camera_pos);
		// camera.setUp(glm::normalize(vec3(0,0,-1)));
		// auto mat_rot = vec_q.toMat4();
		// auto vec3_q = glm::conjugate(vec_q) * glm::vec3(0.0f, 0.0f, -1.0f);
		camera.setRot(rotMatrix);
		// cout<<" camera pos: "<<camera.getPos().x<< " " << camera.getPos().y << " " << camera.getPos().z << endl;

		FPS_counter++;
		currentTime = SDL_GetTicks();
		if (currentTime > lastTime + 1000)
		{
			cout << "FPS :" << FPS_counter << endl;
			lastTime = currentTime;
			FPS_counter = 0;
		}

		display.Clear(0.0f, 0.0f, 0.0f, 0.0f);

		shader.Bind();
		transform.GetPos()->x = km_state.h_move;
		transform.GetPos()->y = km_state.v_move;
		transform.GetPos()->z = km_state.zoom / 4;
		transform.GetRot()->x = km_state.v_rot - 2.0;
		// transform.GetRot()->y = 0;
		transform.GetRot()->z = km_state.h_rot - eulerRot.z + 3.14;
		shader.Update(transform, camera);
		mesh.Draw();

		for (int i = 0; i < axis_v.size(); ++i)
		{
			axis_posMem[i] = rotMatrix * (axis_v[i].pos) + camera_pos;
		}
		axis.Draw();

		auto pcl_colorMem = pcl_mesh.getColorMem();
		auto pcl_PosMem = pcl_mesh.getPosMem();


		if (km_state.pcl)
		{
			for (int i = 0; i < pcl_vertex.size(); ++i)
			{
				pcl_colorMem[i].x = pcl_vertex[i].color.r;
				pcl_colorMem[i].y = pcl_vertex[i].color.g;
				pcl_colorMem[i].z = pcl_vertex[i].color.b;
				pcl_colorMem[i].w = 1.0;

				pcl_PosMem[i].x = pcl_vertex[i].pos.x;
				pcl_PosMem[i].y = pcl_vertex[i].pos.y;
				pcl_PosMem[i].z = pcl_vertex[i].pos.z;
			}
			pcl_mesh.Draw_pcl();
            if(km_state.map_clear)
            {
                pcl_vertex.clear();
                index_accum = 0;
                for (int k = 0; k < pcl_size; ++k)
	            {
		            pcl_vertex.push_back(Vertex(vec3(0, 0, 0),
								vec4(0, 0, 0, 0.0)));
	            }
                km_state.map_clear = false;
            }
		}
		if (km_state.map)
		{
			cube_shader.Bind();
			cube_shader.setCutoff(km_state.probCutoff);
			auto colorMem = cube.getColorMem();

			idx_loop += 0.022;

			for (int index = 0; index < N_total; ++index)
			{
				int N_vertex = 36;
				for (int j = 0; j < N_vertex; ++j)
				{
					colorMem[index * N_vertex + j].x = 0; //mapRGB_data[index].x;
					colorMem[index * N_vertex + j].y = 0; //mapRGB_data[index].y;//sin(10*idx_loop);
					colorMem[index * N_vertex + j].z = 0; //mapRGB_data[index].z;//map_data[index];
					colorMem[index * N_vertex + j].w = 0.0;
				}
			}
			if (test_flag == true)
			{
				int N_vertex = 36;
				for (int index = 0; index < N_total; ++index)
				{
					for (int j = 0; j < N_vertex; ++j)
					{
						colorMem[index * N_vertex + j].x = 0;
						colorMem[index * N_vertex + j].y = 0; //sin(10*idx_loop);
						colorMem[index * N_vertex + j].z = 1; //map_data[index];
						colorMem[index * N_vertex + j].w = sin((float)index / N_total);
					}
					index += 5;
				}
			}
			else
			{
				for (int index = 0; index < N_total; ++index)
				{
					int N_vertex = 36;
					if (map_data[index] > 0.5)
					{
						for (int j = 0; j < N_vertex; ++j)
						{
							colorMem[index * N_vertex + j].x = mapRGB_data[index].x;
							colorMem[index * N_vertex + j].y = mapRGB_data[index].y; //sin(10*idx_loop);
							colorMem[index * N_vertex + j].z = mapRGB_data[index].z; //map_data[index];
							colorMem[index * N_vertex + j].w = map_data[index];
						}
					}
					else
					{
						for (int j = 0; j < N_vertex; ++j)
						{
							colorMem[index * N_vertex + j].x = 0;
							colorMem[index * N_vertex + j].y = 1; //sin(10*idx_loop);
							colorMem[index * N_vertex + j].z = 0; //map_data[index];
							colorMem[index * N_vertex + j].w = 0;
						}
					}
				}
			}

			idx_update++;
			cube_shader.Update(transform, camera);
			// cube.Update_value(color_RGBA, color_RGBA.size());
			cube.Draw_cube();
		}

		display.SwapBuffers();
		SDL_Delay(1);
		// counter += 0.01f;
	}

	return 0;
}
