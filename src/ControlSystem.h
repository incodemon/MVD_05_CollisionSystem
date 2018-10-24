#pragma once
#include "includes.h"
#include "Components.h"
#include <map>

//struct to store mouse state
struct Mouse {
	int x;
	int y;
	int delta_x, delta_y;
};

enum ControlType {
	ControlTypeFree,
	ControlTypeFPS,
	ControlTypeOrbit
};

//System which manages all our controls
class ControlSystem {
public:
	void init();
	void update(float dt);

	//functions called directly from main.cpp, via game
	void updateMousePosition(int new_x, int new_y);
	void key_mouse_callback(int key, int action, int mods);

	//current active control type
	ControlType control_type = ControlTypeFree;

private:
	Mouse mouse_;
	
	float move_speed_ = 10.0f;
	float turn_speed_ = 0.3f;

	bool input[GLFW_KEY_LAST];

	//function to update entity movement
	void updateFree(float dt);
};
