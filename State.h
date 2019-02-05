#ifndef _STATE_H_
#define _STATE_H_
#include <string>

//simulator state for a strategy object
class State
{
public:	
	State(float pack_pos_x, float pack_pos_y, float pack_vel_x, float pack_vel_y,
		  float robot_pos_x, float robot_pos_y, float robot_vel_x, float robot_vel_y,
		  std::string col_obj1,
		  std::string col_obj2):
		   
		pack_pos_x_(pack_pos_x), 
		pack_pos_y_(pack_pos_y), 
		pack_vel_x_(pack_vel_x),
		pack_vel_y_(pack_vel_y),
		
		robot_pos_x_(robot_pos_x), 
		robot_pos_y_(robot_pos_y), 
		robot_vel_x_(robot_vel_x), 
		robot_vel_y_(robot_vel_y)
	{}
		   
	float pack_pos_x_ = 0.0f;
	float pack_pos_y_ = 0.0f;
	float pack_vel_x_ = 0.0f;
	float pack_vel_y_ = 0.0f;
	
	float robot_pos_x_ = 0.0f;
	float robot_pos_y_ = 0.0f;
	float robot_vel_x_ = 0.0f;
	float robot_vel_y_ = 0.0f;
	
	std::string col_obj1_ = "no_object";
	std::string col_obj2_ = "no_object";
};

#endif
