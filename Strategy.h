#ifndef _STRATEGY_H_
#define _STRATEGY_H_
#include <memory>
#include "State.h"

//forward declaration
class AhrSimCore;

//AHR game robot commands
//multiple simultaneous commands are accepted by a simulator
enum ROBOT_DIRECTION
{
	NO_COMMAND    = 0,
	ROBOT_STANDBY = 1,
	ROBOT_LEFT    = 2,
	ROBOT_RIGHT   = 4,
	ROBOT_BACK    = 8,
	ROBOT_FORWARD = 16
};

class Strategy
{
public:		
	//strategy object reacts to a new simultor state and 
	//returns new robot command
	virtual int simulator_state(const State& state) = 0;
	
	virtual ~Strategy() = 0;
};

//typedef std::shared_ptr<Strategy> StrategyPtrType;

class ToyStrategy: public Strategy
{
public:
	ToyStrategy();			
	~ToyStrategy();			
	//provide a strategy with new simulator state
	virtual int simulator_state(const State& state);	
};
#endif
