#ifndef _AHRSIMCORE_H_
#define _AHRSIMCORE_H_

#include "AhrSimApplication.h"
#include "btBulletDynamicsCommon.h"
#include <boost/cstdint.hpp>
#include <boost/unordered_map.hpp>
#include <string>
#include "CState.h"
#include <functional>


typedef boost::unordered_map<boost::uint64_t, std::string> RigidBodyType;

typedef std::function<void(CState)> state_reward_report_cb_type;

//Implemets core AHR simulator logic,
//Handles simulate physical world forces
//by interacting with Bullet engine
//Plays game 
class AhrSimCore : public AhrSimApplication 
{
public:
	AhrSimCore(state_reward_report_cb_type=nullptr);

	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;

	void CreateObjects();

	virtual void Keyboard(unsigned char key, int x, int y) override;
	virtual void KeyboardUp(unsigned char key, int x, int y) override;
	virtual void UpdateScene(float dt);

	virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) override;

	//AHR gane robot commands
	enum ROBOT_DIRECTION
	{
		NO_COMMAND    = 0,
		ROBOT_STANDBY = 1,
		ROBOT_LEFT    = 2,
		ROBOT_RIGHT   = 4,
		ROBOT_BACK    = 8,
		ROBOT_FORWARD = 16
	};

	//Set robot command interface
	void robot_dir_command(ROBOT_DIRECTION direction);

protected:	
	GameObject* m_robot; //AHR robot
	GameObject* m_pack;  //AHR adversarial
	
	//Game board objects
	GameObject* m_b1;
	GameObject* m_b2;
	GameObject* m_b3;
	GameObject* m_b4;
	
	RigidBodyType rigid_objects_;
	
	std::string m_col_obj1 = "no_object"; //name of the first collided object
	std::string m_col_obj2 = "no_object"; //name of the second collided object
	
	boost::uint64_t robot_command_;	

	// a simple trigger volume
	btCollisionObject* pack_gate_; //adversarial gate
	
	btCollisionObject* robot_gate_;	//robot gate

	// keeps track of whether we're holding down the 'g' key
	bool m_bApplyForce;
	
	bool change_robot_vel_;
	
	bool is_sim_initialized_;
	
	state_reward_report_cb_type report_cb_;
};

#endif
