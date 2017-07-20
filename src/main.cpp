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

static const int DISPLAY_WIDTH = 800;
static const int DISPLAY_HEIGHT = 600;

using namespace std;
using namespace glm;

vector<vec3> getPosition(vec3 pos, float voxel_size)
{
	float half_size = voxel_size /2;
	vector<vec3> translate;
	translate.push_back(vec3(-half_size + pos.x, -half_size + pos.y, half_size + pos.z));
	translate.push_back(vec3(-half_size + pos.x, half_size + pos.y, half_size + pos.z));
	translate.push_back(vec3(half_size + pos.x, half_size + pos.y, half_size + pos.z));
	translate.push_back(vec3(half_size + pos.x, -half_size + pos.y, half_size + pos.z));
	translate.push_back(vec3(-half_size + pos.x, -half_size + pos.y, -half_size + pos.z));
	translate.push_back(vec3(-half_size + pos.x, half_size + pos.y, -half_size + pos.z));
	translate.push_back(vec3(half_size + pos.x, half_size + pos.y, -half_size + pos.z));
	translate.push_back(vec3(half_size + pos.x, -half_size + pos.y, -half_size + pos.z));
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
	for(int i = 0; i < N.x; ++i){
		for(int j = 0; j < N.y; ++j){
			for(int k = 0; k < N.z; ++k){
			pos = getPosition(vec3(range.x * (float)i/N.x + min.x,
				range.y * (float)j/N.y + min.y, range.z * (float)k/N.z + min.z), grid_size);
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

template <typename T>
class sub_base
{
public:
    T msg;
	bool draw = false;
	int N_x, N_y, N_z;
	inline bool getDrawFlag(){return draw;};
    std::vector<Eigen::Vector3i> vao_data;
    void callback(const typename T::ConstPtr& msg_sub)
    {
		cout<<msg.layout.dim[0].size<<endl;
        msg = *msg_sub;
		N_x = msg.layout.dim[0].size;
		N_y = msg.layout.dim[1].size;
		N_z = msg.layout.dim[2].size;
		draw = true;
    };
};

class sub_map:public sub_base<std_msgs::Float64MultiArray>{

};

bool map_flag = false, mapRGB_flag = false;
int N_x, N_y, N_z, N_total;
std::vector<double> map_data;
std::vector<vec3> mapRGB_data;

void mapCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%d]", msg->layout.dim[0].size);
	if(!map_flag)
	{
		N_x = msg->layout.dim[0].size;
		N_y = msg->layout.dim[1].size;
		N_z = msg->layout.dim[2].size;
		int Ntotal = N_x * N_y * N_z;
		cout<<"Dimentions: "<<N_x<<"x"<<N_y<<"x"<<N_z<<endl;
		map_data.reserve(Ntotal);
	}
	map_flag = true;
	for(int i = 0; i<N_total; ++i)
	{
		map_data[i] = msg->data[i];
	}

}

void mapRGBCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%d]", msg->layout.dim[0].size);
  	int Ntotal = 0;
	if(!mapRGB_flag)
	{
		int Nx = msg->layout.dim[0].size;
		int Ny = msg->layout.dim[1].size;
		int Nz = msg->layout.dim[2].size;
		Ntotal = Nx * Ny * Nz;
		cout<<"RGB map dimentions: "<<Nx<<"x"<<Ny<<"x"<<Nz<<endl;
		mapRGB_data.reserve(Ntotal);
	}
	mapRGB_flag = true;
	for(int i = 0; i<N_total; ++i)
	{
		mapRGB_data[i].x = (float)std::max(0, (int)msg->data[i*3])/255;
		mapRGB_data[i].y = (float)std::max(0, (int)msg->data[i*3 + 1])/255;
		mapRGB_data[i].z = (float)std::max(0, (int)msg->data[i*3 + 2])/255;
	}

}

// void update(auto& vbo_data, int N_voxel){
//     for(int index = 0; index < N_voxel; ++index)
//     {
//       vao_data.push_back(msg.data[index]);
//     }
// }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visualizer");
	ros::NodeHandle nh;
	ros::Subscriber mapProbSub, mapRGBSub;
	sub_map mapProbListener;
    // sub_base<std_msgs::Int16MultiArray> mapRGBListener;
	// mapRGBSub = nh.subscribe
    //  ("/map_rgb",   1, &sub_base<std_msgs::Int16MultiArray>::callback, &mapRGBListener);
    mapProbSub = nh.subscribe("/map_probabilities", 10, mapCallback);
	mapRGBSub = nh.subscribe("/map_rgb", 10, mapRGBCallback);
	cout<<"waiting for map msgs...."<<endl;
	cout<<"size of mapRGB: "<<mapRGB_data.size()<<endl;

	float grid_size = 0.075;
	// unsigned int N_x, N_y,N_z;
	float L_x, L_y, L_z;
	float x_min, y_min, z_min;

	Display display(DISPLAY_WIDTH, DISPLAY_HEIGHT, "OpenGL");
	std::vector<Vertex> vert_vec;
	std::vector<unsigned int> idx_vec;

	// construct simple test case
	bool test_flag = false;
	if(argc==2)
	{
		test_flag = true;
		cout<<"test value: "<<argv[1]<<endl;
		grid_size = 0.5;
		L_x = 10, L_y = 10, L_z = 5;
		N_x = L_x/grid_size;
		N_y = L_y/grid_size;
		N_z = L_z/grid_size;
		x_min = -L_x/2, y_min = -L_y/2, z_min = 0;
		N_total = N_z * N_y * N_x;
	}
	else
	{
		while(!map_flag || !mapRGB_flag)
		{
			ros::spinOnce();
		}
		cout<<"size of mapRGB: "<<mapRGB_data.size()<<endl;
		cout<<"size of map: "<<map_data.size()<<endl;
		std::cout << "Have " << argc << " arguments:" << std::endl;
	    for (int i = 1; i < argc; ++i) {
	        std::cout << atof(argv[i]) << ", ";
    	}
		L_x = N_x * grid_size;
		L_y = N_y * grid_size;
		L_z = N_z * grid_size;
		x_min = -L_x/2, y_min = -L_y/2, z_min = 0;
	}
	if(argc==8)
	{
		x_min = atof(argv[1]), y_min = atof(argv[2]), z_min = atof(argv[3]);
		grid_size = atof(argv[4]);
		L_x = atof(argv[5]), L_y = atof(argv[6]), L_z = atof(argv[7]);
	}

	N_total = N_z * N_y * N_x;
	cout<<"Voxel total count: "<<N_total<<endl;
	float x_max = x_min + L_x;
	float y_max = y_min + L_y;
	float z_max = z_min + L_z;

	vec3 N_dim(N_x,N_y,N_z);
	// int N = round(L_x/grid_size);
	// int N_z = 1;
	// std::vector<glm::vec3> voxel_pos;
	// voxel_pos.reserve(N_dim.z*N_dim.y*N_dim.x);
	// for(int k = 0; k < N_dim.z; ++k)
	// {
	// 	for(int i = 0; i < N_dim.y; ++i){
	// 		for(int j = 0; j < N_dim.x; ++j)
	// 		{
	// 			voxel_pos.push_back(glm::vec3(L_y * (float)j/N_dim.x + x_min, L_x * (float)i/N_dim.y + y_min, k*0.5));
	// 		}
	// 	}
	// }

	Vertex vertex(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0,0), glm::vec3(0.0f,0.0f,1.0f));
	vertex.pos.x = x_max;
	vertex.pos.y = y_max;
	vertex.pos.z = z_max;
	vert_vec.push_back(vertex);
	vertex.pos.x = x_min;
	vert_vec.push_back(vertex);vert_vec.push_back(vertex);
	vertex.pos.y = y_min;
	vert_vec.push_back(vertex);vert_vec.push_back(vertex);
	vertex.pos.x = x_max;
	vert_vec.push_back(vertex);vert_vec.push_back(vertex);
	vertex.pos.y = y_max;
	vert_vec.push_back(vertex);


	vertex.pos.z = z_min;
	for(int i = 0; i < N_dim.x + 1 ; ++i)
	{
		vertex.pos.y = y_min;
		vertex.pos.x = L_x * (float)i/N_dim.x + x_min;
		vert_vec.push_back(vertex);
		vertex.pos.y = L_y + y_min;
		vert_vec.push_back(vertex);

		vertex.pos.x = x_min;
		vertex.pos.y = L_y * (float)i/N_dim.y + y_min;
		vert_vec.push_back(vertex);
		vertex.pos.x = L_x + x_min;
		vert_vec.push_back(vertex);

	}

	for(unsigned int i = 0 ; i < vert_vec.size() ; i+=2)
	{
		auto idx = i;
		idx_vec.push_back(idx);
		idx_vec.push_back(idx+1);
	}


	colorcube(vec3(x_min, y_min, z_min), vec3(L_x, L_y, L_z), grid_size);
	// Eigen::Vector3d pos_min;
	// pos_min << x_min, y_min, z_min;

	// auto ray_cast  = Ray_base(x_min, y_min, z_min, L_x, L_y, L_z, grid_size);
	//
	// Eigen::Vector3d ray_start(6,1,1);
	// Eigen::Vector3d ray_end(1,9,4);
	// cout<<"ending ray: \n"<<ray_end<<endl;
 //  	ray_cast.setStartEnd(ray_start, ray_end);
	//
	// cout<<"ray voxel simple: "<<ray_cast.SingleRayCasting3D()<<endl;
	// // for(int index = 0 ; index < ray_cast.SingleRayCasting3D(); ++index)
	// // {
	// // 	cout<<<<endl;
	// // }
	// cout<<"ray voxel ADD: "<<ray_cast.voxel_traversal()<<endl;
	//
	// auto visited = ray_cast.getVisited();
	// cout<<" size: "<<visited.size()<<endl;
	//
	// Eigen::Vector3i point;
	// for(unsigned int j= 0; j< visited.size(); ++j){
	// 	// cout<<typeid(visited).name()<<endl;
	// 	try{
	// 		point = visited[j];
	//
	// 	// cout<<"index: "<< j << " val: " <<point[2]<<endl;
	// 	// cout<<point<<endl;
	// 	}
	// 	catch(std::bad_alloc& ba)
	// 	{
	// 		std::cerr << "bad mem alloc "<<ba.what()<<'\n';
	// 	}
	// 		// int index = N*point[0] +  N*point[1] +  point[2];
	// 		// cout<<point[0]<< " " <<point[1]<<" "<<point[2]<<" * "<<endl;
	// }
		// for(auto voxel : visited)
		// {
		// 	int index = voxel[0] +  N*voxel[1] +  N*N*voxel[2];
		// 	cout<<index<<endl;
		// }

	Mesh mesh(vert_vec, vert_vec.size(), idx_vec, idx_vec.size());
	Mesh cube(cube_vertex, cube_vertex.size(), cube_index, cube_index.size());
	//Mesh monkey("./res/monkey3.obj");
	// cout<<ros::package::getPath("map_viz")<<endl;
	Shader shader("./res/basicShader");
	Shader cube_shader("./res/cubeShader");
	Texture texture("./res/bricks.jpg");
	Transform transform;
	Camera camera(glm::vec3(0.0f, 2.0f, -15.0f), 70.0f,
		(float)DISPLAY_WIDTH/(float)DISPLAY_HEIGHT, 0.1f, 1000.0f);
	// vector<vec4> color_RGBA;
	// color_RGBA.reserve(cube_vertex.size());
	// int N_v = cube_vertex.size();
	// for(unsigned int i = 0; i < cube_vertex.size(); ++i)
	// {
	// 	color_RGBA.push_back(vec4((float)i/N_v,1.0 - (float)i/N_v,1.0,1.0));
	// }

	cout<<cube_vertex.size()<<'\n';
	// cout<<color_RGBA.size()<<'\n';

	SDL_Event event;
	auto isRunning = true;
	auto counter = 0.0f;
	float z_key = 0.0, trans_key = 0.0;
	float rot_v = 0;
	double idx_loop = 0;
	int idx_update = 0;
	unsigned int lastTime = 0, currentTime;
	unsigned int FPS_counter = 0;
	while(isRunning)
	{
		ros::spinOnce();
		while(SDL_PollEvent(&event))
		{
			if(event.type == SDL_QUIT)
				isRunning = false;
			SDL_PollEvent(&event);
			switch(event.type)
			{
			case SDL_KEYUP:
			case SDL_KEYDOWN:
				// printf("'%c' was %s \n", event.key.keysym.sym,
					// (event.key.state == SDL_PRESSED) ? "pressed" : "released");
				// SDLKey keyPressed = e.key.keysym.sym;
				switch(event.key.keysym.sym)
				{
				case SDLK_LEFT:
					counter-=0.1;
					break;
				case SDLK_RIGHT:
					counter+=0.1;
					break;
				case SDLK_UP:
					rot_v-=0.1;
					break;
				case SDLK_DOWN:
					rot_v+=0.1;
					break;
				case SDLK_w:
					z_key-=2;
					break;
				case SDLK_s:
					z_key+=2;
					break;
				case SDLK_a:
					cout<<"key pressed\n";
					trans_key-=2;
					break;
				case SDLK_d:
					trans_key+=2;
					break;
				case SDLK_q:
					exit(0);
					break;
				}
				break;
			}
		}
		FPS_counter ++;
		currentTime = SDL_GetTicks();
		if (currentTime > lastTime + 1000)
		{
			cout<<"FPS :"<<FPS_counter<<endl;
			lastTime = currentTime;
			FPS_counter = 0;
		}



		display.Clear(0.0f, 0.0f, 0.0f, 1.0f);

		// auto sinCounter = sinf(counter);
		// auto absSinCounter = std::abs(sinCounter);

		//transform.GetPos()->x = sinCounter;
		transform.GetRot()->y = 0;
		//transform.GetRot()->z = counter * 100;
		//transform.GetScale()->x = absSinCounter;
		//transform.GetScale()->y = absSinCounter;

		shader.Bind();
		texture.Bind();
		transform.GetPos()->x = trans_key;
		transform.GetPos()->y = 0;
		transform.GetPos()->z = z_key/4;
		transform.GetRot()->x = -90 + rot_v;
	 	transform.GetRot()->y = 0;
		transform.GetRot()->z = counter;
		shader.Update(transform, camera);
		// monkey.Draw();
		mesh.Draw();

		cube_shader.Bind();
		auto colorMem = cube.getColorMem();

		if(idx_update%50 == 0)
		{
		// for(int index = 0; index < color_RGBA.size(); ++index)
		// for(unsigned int index = 0; index< color_RGBA.size() && idx_loop == 0; ++index)
		// {
		// 	// cout<<"test"<<endl;
		// 	// cout<<visited[k]<<endl;
		//
		// 	// index = N*N*index +  N*index +  index;
		// 	// index = N*N*index +  N*index +  index;
		// 	// color_RGBA[i].w = (sin(counter * 10)+1)/2;
		// 	colorMem[index].x = 0;
		// 	colorMem[index].y = 0;
		// 	colorMem[index].z = 0;//(sin(counter * 10)+1)/2;
		// 	// if(index%N == 0){
		// 	colorMem[index].w = 0;
		// 	// }else{
		// 	// 	colorMem[index].w = 0.0;
		// 	// }
		// }

		// ray_start << 0,0,2.5;
		// ray_end << 2.5*cos(idx_loop), 2.5*sin(idx_loop), 2.5 + sin(5*idx_loop);
		idx_loop += 0.022;
		// ray_cast.setStartEnd(ray_start, ray_end);

		// visited = ray_cast.getVisited();
		// for(auto voxel : visited)
		// {
		// 	int index = 36*(voxel[0] +  N*voxel[1] +  N*N*voxel[2]);
		// 	for(int j= 0; j < 36; ++j)
		// 	{
		// 	colorMem[index + j].x = 0;
		// 	colorMem[index + j].y = 1;
		// 	colorMem[index + j].z = 0;
		// 	colorMem[index + j].w = 1;
		// 	}
		// }


	// int N_single = ray_cast.SingleRayCasting3D();
	// auto single_ray = ray_cast.getIndices();

		// for(int k = 0 ; k < N_single; ++k)
		for(int index = 0; index < N_total; ++index)
		{
			// cout<<single_ray[k]<<endl;
			// int index = single_ray[k];
			// cout<<"single index: "<<index<<endl;
			// Eigen::Vector3i position = ray_cast.IndToSub(index);
			// vector<int> pos_int(3);
			// for(int j = 0; j<3; ++j)
			// {
			// 	pos_int[j] = floor((position[j] - pos_min[j])/ray_cast.getBinSize());
			// }
			// index = 36*(position[0] +  N_dim.x*position[1] +  N_dim.y*N_dim.x*position[2]);
			int N_vertex = 36;
			if(map_data[index] > 0.5)
			{
				for(int j= 0; j < N_vertex; ++j)
				{
					colorMem[index*N_vertex + j].x = mapRGB_data[index].x;
					colorMem[index*N_vertex + j].y = mapRGB_data[index].y;//sin(10*idx_loop);
					colorMem[index*N_vertex + j].z = mapRGB_data[index].z;//map_data[index];
					colorMem[index*N_vertex + j].w = 1.0;
				}
			}else
			{
				for(int j= 0; j < N_vertex; ++j)
				{
					colorMem[index*N_vertex + j].w = 0.0;
				}
			}
		}
		if(test_flag == true)
		{
			int N_vertex = 36;
			for(int index = 0 ; index < 100; ++index)
			{
				for(int j= 0; j < N_vertex; ++j)
				{
					colorMem[index*N_vertex + j].x = 0;
					colorMem[index*N_vertex + j].y = 0;//sin(10*idx_loop);
					colorMem[index*N_vertex + j].z = 1;//map_data[index];
					// colorMem[index*N_vertex + j].w = 1;
					colorMem[index*N_vertex + j].w = 1.0;
				}
			}
		}
		}
	idx_update++;
		cube_shader.Update(transform, camera);
		// cube.Update_value(color_RGBA, color_RGBA.size());
		cube.Draw_cube();

		display.SwapBuffers();
		SDL_Delay(1);
		// counter += 0.01f;
	}

	return 0;
}
