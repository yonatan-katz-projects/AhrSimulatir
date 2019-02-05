#include "Strategy.h"
#include <iostream>

using namespace std;

Strategy::~Strategy()
{}

ToyStrategy::ToyStrategy()
{}

ToyStrategy::~ToyStrategy()
{}

int ToyStrategy::simulator_state(const State& s)
{
	float delta_x = s.pack_pos_x_ - s.robot_pos_x_;
	
	float delta_y = s.pack_pos_y_ - s.robot_pos_y_;		
	
	int robot_command = (int)ROBOT_DIRECTION::NO_COMMAND;
	     
	if (delta_y >5 && s.robot_pos_y_<20)
	{		
		robot_command |= (int)ROBOT_DIRECTION::ROBOT_LEFT;	
	}
	else if (delta_y<-5 && s.robot_pos_y_>-20)
	{	
		robot_command |= (int)ROBOT_DIRECTION::ROBOT_RIGHT;		
	}	
	
	if (delta_x>-20 && s.robot_pos_x_ >30)
	{		
		robot_command |= (int)ROBOT_DIRECTION::ROBOT_FORWARD;			
	}
	else if (s.robot_pos_x_>25 && s.robot_pos_x_<40)
	{
		robot_command |= (int)ROBOT_DIRECTION::ROBOT_BACK;			
	}	
	
	return robot_command;	
}


