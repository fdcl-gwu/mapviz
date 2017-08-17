#ifndef K_M_H
#define K_M_H
#include <iostream>
#include <algorithm>
#include "SDL2/SDL.h"

using namespace std;

struct key_mouse_state
{
    bool isRunning = true;
    float v_rot = 0; // vertical rotation
    float h_rot = 0;
    float v_move = 0; // vertical translation
    float h_move = 0;
    float zoom = 0;
    float probCutoff = 0.8;
	bool map = true;
    bool map_clear = false;
  bool pcl = false;
};

void SDL_event_handle(SDL_Event& event, key_mouse_state& control)
{
	while(SDL_PollEvent(&event))
	{
		if(event.type == SDL_QUIT)
			control.isRunning = false;
		SDL_PollEvent(&event);
		switch(event.type)
		{
		case SDL_KEYUP:
		case SDL_KEYDOWN:
			// printf("'%c' was %s \n", event.key.keysym.sym,
			// 	(event.key.state == SDL_PRESSED) ? "pressed" : "released");
			// SDLKey keyPressed = e.key.keysym.sym;
			switch(event.key.keysym.sym)
			{
			case SDLK_LEFT:
				control.h_rot-=0.1;
				break;
			case SDLK_RIGHT:
				control.h_rot+=0.1;
				break;
			case SDLK_UP:
				control.v_rot-=0.1;
				break;
			case SDLK_DOWN:
				control.v_rot+=0.1;
				break;
			case SDLK_w:
				control.zoom-=2;
				break;
			case SDLK_s:
				control.zoom+=2;
				break;
			case SDLK_a:
				control.h_move-=2;
				break;
			case SDLK_d:
				control.h_move+=2;
				break;
			case SDLK_ESCAPE:
				control.isRunning = false;
				break;
			case SDLK_r:
				control.v_move+=2;
				break;
			case SDLK_f:
				control.v_move-=2;
				break;
            case SDLK_p:
				control.pcl = !control.pcl;
                break;
            case SDLK_c:
				control.map_clear = true;
                break;
			case SDLK_m:
				control.map = !control.map;
				break;
			case SDLK_j:
				if(control.probCutoff < 0.9)
					control.probCutoff += 0.1;
				else
					control.probCutoff = 1.0;
				cout<<"prob cutoff: "<<control.probCutoff<<endl;
				break;
			case SDLK_k:
				if(control.probCutoff > 0.1)
					control.probCutoff -= 0.1;
				else
					control.probCutoff = 1e-5;
				cout<<"prob cutoff: "<<control.probCutoff<<endl;
				break;
			}
			break;
		}
	}
}


#endif
