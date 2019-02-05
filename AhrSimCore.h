#ifndef _AHRSIMCORE_H_
#define _AHRSIMCORE_H_

#include "AhrSimApplication.h"
#include "btBulletDynamicsCommon.h"
#include <boost/cstdint.hpp>
#include <boost/unordered_map.hpp>
#include <string>
#include "State.h"
#include "Strategy.h"
#include <functional>


typedef boost::unordered_map<boost::uint64_t, std::string> RigidBodyType;

//Implements core AHR simulator logic,
//Handles physical world forces simulation
//by interacting with Bullet engine
//Plays air hockey game according to a robot strategy
class AhrSimCore : public AhrSimApplication 
{
public:
	AhrSimCore(Strategy* robot_strategy);

	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;

	void CreateObjects();

	//Override strategy robot command, just for debug!
	virtual void Keyboard(unsigned char key, int x, int y) override;
	virtual void KeyboardUp(unsigned char key, int x, int y) override;
	virtual void UpdateScene(float dt);
	virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) override;	

protected:	
	GameObject* m_robot; //AHR robot
	GameObject* m_pack;  //AHR adversarial
	
	//Game board objects
	GameObject* m_b1;
	GameObject* m_b2;
	GameObject* m_b3;
	GameObject* m_b4;
	
	RigidBodyType m_rigid_objects;
	
	std::string m_col_obj1 = "no_object"; //name of the first collided object
	std::string m_col_obj2 = "no_object"; //name of the second collided object
	
	boost::uint64_t m_robot_command;	

	// a simple trigger volume
	btCollisionObject* m_pack_gate; //adversarial gate
	
	btCollisionObject* m_robot_gate;	//robot gate

	// keeps track of whether we're holding down the 'g' key
	bool m_bApplyForce;
	
	bool m_change_robot_vel;
	
	bool m_is_sim_initialized;
	
	Strategy* m_robot_strategy = 0;
};

#endif
