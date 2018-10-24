//
//  Game.cpp
//
//  Copyright � 2018 Alun Evans. All rights reserved.
//

#include "Game.h"
#include "Shader.h"
#include "extern.h"
#include "Parsers.h"

Game::Game() {

}

//In this example we simply fire a ray in the -z direction and, in CollisionSystem.cpp, print of it collides or not
void Game::init() {


	//******* INIT SYSTEMS *******//

	//init systems except debug, which needs info about scene
	control_system_.init();
	graphics_system_.init();

    //******** CREATE AND ADD RESOURCES **********//
    
    //shaders
	graphics_system_.loadShader("phong", "data/shaders/phong.vert", "data/shaders/phong.frag");
    
    //geometries
    int floor_geom_id = graphics_system_.createGeometryFromFile("data/assets/floor_5x5.obj");
    int plane_geom_id = graphics_system_.createPlaneGeometry();
    
    //materials and textures
    int default_mat_id = graphics_system_.createMaterial();
	graphics_system_.getMaterial(default_mat_id).diffuse_map = Parsers::parseTexture("data/assets/block_blue.tga");;
	graphics_system_.getMaterial(default_mat_id).shader_id = graphics_system_.getShaderProgram("phong");
	graphics_system_.getMaterial(default_mat_id).specular = lm::vec3(0, 0, 0);

    
    //******* CREATE ENTITIES AND ADD COMPONENTS *******//


	//floor
    int ent_wall_near = ECS.createEntity("Wall Near");
    //Mesh& fmc = ECS.createComponentForEntity<Mesh>(ent_wall_near);
    //fmc.geometry = floor_geom_id; fmc.material = default_mat_id;
    ECS.getComponentFromEntity<Transform>(ent_wall_near).rotateLocal(-90.0f*DEG2RAD, lm::vec3(1.0f, 0.0f, 0.0f)); //rotate it flat
    ECS.getComponentFromEntity<Transform>(ent_wall_near).translate(0, 0, -10);
    Collider& box_collider = ECS.createComponentForEntity<Collider>(ent_wall_near);
    box_collider.collider_type = ColliderTypeBox;
    box_collider.local_center = lm::vec3(-2.5f, -0.125f, -2.5f);
    box_collider.local_halfwidth = lm::vec3(2.5f, 0.125f, 2.5f);
    
    //floor
    int ent_wall_far = ECS.createEntity("Wall Far");
    Mesh& wfmc = ECS.createComponentForEntity<Mesh>(ent_wall_far);
    wfmc.geometry = floor_geom_id; wfmc.material = default_mat_id;
    ECS.getComponentFromEntity<Transform>(ent_wall_far).rotateLocal(-90.0f*DEG2RAD, lm::vec3(1.0f, 0.0f, 0.0f)); //rotate it flat
    ECS.getComponentFromEntity<Transform>(ent_wall_far).translate(2, 0, -20);
    Collider& box_collider_far = ECS.createComponentForEntity<Collider>(ent_wall_far);
    box_collider_far.collider_type = ColliderTypeBox;
    box_collider_far.local_center = lm::vec3(-2.5f, -0.125f, -2.5f);
    box_collider_far.local_halfwidth = lm::vec3(2.5f, 0.125f, 2.5f);

	
    int ent_light = ECS.createEntity("Light 1");
    ECS.createComponentForEntity<Light>(ent_light);
    ECS.getComponentFromEntity<Transform>(ent_light).translate(100.0f, 100.0f, 100.0f);
    ECS.getComponentFromEntity<Light>(ent_light).color = lm::vec3(1.0f, 1.0f, 1.0f);
    
    
    //player and camera
    int ent_player = ECS.createEntity("Player");
    Camera& player_cam = ECS.createComponentForEntity<Camera>(ent_player);
    ECS.getComponentFromEntity<Transform>(ent_player).translate(-2.0f, 3.0f, -2.0f);
    player_cam.position = lm::vec3(-2.0f, 3.0f, -2.0f);
    player_cam.forward = lm::vec3(0.0f, 0.0f, -1.0f);
    player_cam.setPerspective(60.0f*DEG2RAD, 1, 0.1f, 10000.0f);

    
    ECS.main_camera = ECS.getComponentID<Camera>(ent_player);

    //collider ray
    int ent_forward_ray = ECS.createEntity("Forward Ray");
    Transform& forward_ray_trans = ECS.createComponentForEntity<Transform>(ent_forward_ray);
    forward_ray_trans.translate(0, -1, 0);
    forward_ray_trans.parent = ECS.getComponentID<Transform>(ent_player); //set parent as player entity *transform*!
    Collider& forward_ray_collider = ECS.createComponentForEntity<Collider>(ent_forward_ray);
    forward_ray_collider.collider_type = ColliderTypeRay;
    forward_ray_collider.direction = lm::vec3(0.0,0.0,-1.0);
    forward_ray_collider.max_distance = 100.0f;
    
    //******* INIT DEBUG SYSTEM *******//
    debug_system_.init();
    debug_system_.setActive(true);
}

//Entry point for game update code
void Game::update(float dt) {
	//update each system in turn

	//update input
	control_system_.update(dt);

	//collision
	collision_system_.update(dt);

	//render
	graphics_system_.update(dt);
    
    //debug
    debug_system_.update(dt);
    

	

}

