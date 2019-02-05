#include "AhrSimCore.h"
#include <iostream>

using namespace std;

AhrSimCore::AhrSimCore(Strategy* robot_strategy) :
	AhrSimApplication(),
	m_robot_command(ROBOT_STANDBY),	
	m_bApplyForce(false),
	m_change_robot_vel(false),
	m_is_sim_initialized(false),
	m_robot_strategy(robot_strategy)
{}

void AhrSimCore::InitializePhysics() 
{
	// create the collision configuration
	m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the constraint solver
	m_pSolver = new btSequentialImpulseConstraintSolver();
	// create the world
	m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);

	// create our scene's physics objects
	CreateObjects();
}

void AhrSimCore::ShutdownPhysics() {
	delete m_pWorld;
	delete m_pSolver;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;
}

btCollisionObject* create_trigger_gate(btVector3 dim, btVector3 pos)
{
	// create a trigger volume
	btCollisionObject* m_pTrigger = new btCollisionObject();
	// create a box for the trigger's shape
	//m_pTrigger->setCollisionShape(new btBoxShape(btVector3(1,0.25,1)));
	m_pTrigger->setCollisionShape(new btBoxShape(dim));
	// set the trigger's position
	btTransform triggerTrans;
	triggerTrans.setIdentity();
	//triggerTrans.setOrigin(btVector3(0,1.5,0));
	triggerTrans.setOrigin(pos);
	m_pTrigger->setWorldTransform(triggerTrans);
	// flag the trigger to ignore contact responses
	m_pTrigger->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	// add the trigger to our world
	return m_pTrigger;
	
	
}

void AhrSimCore::CreateObjects() {
	// create a ground plane
	CreateGameObject(new btBoxShape(btVector3(1,50,30)), 0, btVector3(0.2f, 0.6f, 0.6f), btVector3(0.0f, 0.0f, 0.0f));			
	
	m_b1 = CreateGameObject(new btBoxShape(btVector3(10,50,1)), 100, btVector3(0.7f, 0.7f, 0.0f), btVector3(0.0f, 0.0f, 30.0f));			
	m_b1->GetRigidBody()->setLinearFactor(btVector3(0,0,0));	
	m_b1->GetRigidBody()->setAngularFactor(btVector3(0,0,0));
	
	m_b2 = CreateGameObject(new btBoxShape(btVector3(10,50,1)), 100, btVector3(0.7f, 0.7f, 0.0f), btVector3(0.0f, 0.0f, -30.0f));	
	m_b2->GetRigidBody()->setLinearFactor(btVector3(0,0,0));	
	m_b2->GetRigidBody()->setAngularFactor(btVector3(0,0,0));
	
	//pack and robot sides
	m_b3 = CreateGameObject(new btBoxShape(btVector3(1,30,10)), 100, btVector3(0.7f, 0.7f, 0.0f), btVector3(-50.0f, 0.0f, 0.0f), btQuaternion(0,1,1,0));	
	m_b3->GetRigidBody()->setLinearFactor(btVector3(0,0,0));	
	m_b3->GetRigidBody()->setAngularFactor(btVector3(0,0,0));
	
	m_b4 = CreateGameObject(new btBoxShape(btVector3(1,30,10)), 100, btVector3(0.7f, 0.7f, 0.0f), btVector3(50.0f, 0.0f, 0.0f), btQuaternion(0,1,1,0));	
	m_b4->GetRigidBody()->setLinearFactor(btVector3(0,0,0));	
	m_b4->GetRigidBody()->setAngularFactor(btVector3(0,0,0));
	
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_b1->GetRigidBody())] = "b1";
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_b2->GetRigidBody())] = "b2";
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_b3->GetRigidBody())] = "b3";
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_b4->GetRigidBody())] = "b4";


	// create a green cylinder	
	m_robot = CreateGameObject(
		new btCylinderShape(btVector3(3,1.25,3)), 
			10.0f, btVector3(0.0f, 0.7f, 0.0f)/*color*/, 
			btVector3(40, 2.0f, 0.0f), 
			btQuaternion(0,0,0,1));
	m_robot->GetRigidBody()->setLinearFactor(btVector3(1,0,1));	
	m_robot->GetRigidBody()->setAngularFactor(btVector3(1,0,1));
	
	
	m_pack = CreateGameObject(
		new btCylinderShape(btVector3(3,1.25,3)), 
			1.0f, btVector3(2.0f, 0.2f, 0.2f)/*color*/, 
			btVector3(-40, 1.3f, 10.0f), 			
			btQuaternion(0,0,0,1));
			
	m_pack->GetRigidBody()->setLinearFactor(btVector3(1,0,1));	
	m_pack->GetRigidBody()->setAngularFactor(btVector3(1,0,1));	
	
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_pack->GetRigidBody())] = "pack";
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_robot->GetRigidBody())] = "robot";
	
	
	//make make gtriger gates
	m_robot_gate = create_trigger_gate(
		btVector3(1,11,11),//shape 
		btVector3(48,1,0));		
	m_pWorld->addCollisionObject(m_robot_gate);//pos
			
	//make make gtriger gates
	m_pack_gate = create_trigger_gate(
		btVector3(1,11,11),//shape 
		btVector3(-48,1,0));		
	m_pWorld->addCollisionObject(m_pack_gate);//pos
	
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_robot_gate)] = "robot_gate";
	m_rigid_objects[reinterpret_cast<boost::uint64_t>(m_pack_gate)] =  "pack_gate";
	
	m_is_sim_initialized = true;
}

void AhrSimCore::CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) 
{	
	if (m_is_sim_initialized)
	{
		RigidBodyType::iterator it1 = m_rigid_objects.find(reinterpret_cast<boost::uint64_t>(pBody0)); 
		RigidBodyType::iterator it2 = m_rigid_objects.find(reinterpret_cast<boost::uint64_t>(pBody1)); 
		
		if (it1 != m_rigid_objects.end()) m_col_obj1 = (std::string)(it1->second);	
		if (it2 != m_rigid_objects.end()) m_col_obj1 = (std::string)(it2->second);		
	}
}


void AhrSimCore::Keyboard(unsigned char key, int x, int y) 
{
	// call the base implementation first
	AhrSimApplication::Keyboard(key, x, y);	
	
	switch(key) 
	{	
		case 'h':
		{			
			m_robot->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
			m_robot_command = ROBOT_LEFT;
			m_change_robot_vel = true;
			break;
		}
		case 'j':
		{		
			m_robot->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
			m_robot_command = ROBOT_RIGHT;
			m_change_robot_vel= true;
			break;
		}
		case 'u':
		{		
			m_robot->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
			m_robot_command = ROBOT_FORWARD;
			m_change_robot_vel= true;
			break;
		}
		case 'n':
		{		
			m_robot->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
			m_robot_command = ROBOT_BACK;
			m_change_robot_vel = true;
			break;
		}		
		case 's':
		{		
			m_robot->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
			m_robot_command = ROBOT_STANDBY;			
			m_change_robot_vel = true;
			break;
		}
		case 'f':
		{
			m_pack->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
			m_bApplyForce = true;
			break;			
		}
	}
	
}

void AhrSimCore::KeyboardUp(unsigned char key, int x, int y) 
{	
}

void AhrSimCore::UpdateScene(float dt) {
	// call the base implementation first
	AhrSimApplication::UpdateScene(dt);
	int left_right = 0;
	int back_forward = 0;	

	if (m_change_robot_vel == true)
	{
		// Force testing
		if (m_robot_command & ROBOT_LEFT)
		{	
			left_right = 80;
		}
		else if (m_robot_command & ROBOT_RIGHT)
		{		
			left_right = -80;						
		}		
		
		if(m_robot_command & ROBOT_BACK)
		{	
			back_forward = 80;						
		}
		else if(m_robot_command & ROBOT_FORWARD)
		{
			back_forward = -80;			
		}		
		
		if(m_robot_command & ROBOT_STANDBY)
		{	
			back_forward = left_right = 0;
		}		
		m_change_robot_vel = false;		
		m_robot_command = ROBOT_STANDBY;		
	}
	
	if (m_bApplyForce)
	{
		m_bApplyForce  = false;
		
		m_pack->GetRigidBody()->applyCentralImpulse(btVector3(100, 0, 50));
	}
	
	
	m_robot->GetRigidBody()->setLinearVelocity(btVector3(back_forward, 0, left_right));	
	m_robot->GetRigidBody()->forceActivationState(ACTIVE_TAG); 
	m_pack->GetRigidBody()->forceActivationState(ACTIVE_TAG); 
	m_b1->GetRigidBody()->forceActivationState(ACTIVE_TAG); 
	m_b2->GetRigidBody()->forceActivationState(ACTIVE_TAG); 
	m_b3->GetRigidBody()->forceActivationState(ACTIVE_TAG); 
	m_b4->GetRigidBody()->forceActivationState(ACTIVE_TAG); 	
	 
		 
	if (m_robot_strategy)
	{
		btTransform trans_pack;
		m_pack->GetRigidBody()->getMotionState()->getWorldTransform(trans_pack);
		btVector3 v_pack = m_pack->GetRigidBody()->getLinearVelocity();
		
		btTransform trans_robot;
		m_robot->GetRigidBody()->getMotionState()->getWorldTransform(trans_robot);
		btVector3 v_robot = m_robot->GetRigidBody()->getLinearVelocity();				
		
		//Raycast for the pack object
		btVector3 btFrom = trans_pack.getOrigin();
		btVector3 btTo = btFrom + v_pack * 100; //pack velocity vector
		
		btCollisionWorld::AllHitsRayResultCallback RayCallback(btFrom, btTo);		
		m_pWorld->rayTest(btFrom, btTo, RayCallback);
		
		m_pDebugDrawer->drawLine(btFrom, btTo, btVector3(1.0f, 0.7f, 0.0f));				

		if(RayCallback.hasHit())
		{
			//take the first collision object
			const btCollisionObject* collision_object = RayCallback.m_collisionObjects[0];			
			
			RigidBodyType::iterator it = m_rigid_objects.find(reinterpret_cast<boost::uint64_t>(collision_object)); 		
			
			if (it != m_rigid_objects.end())
			{			
				std::string col_raycast = (std::string)(it->second);
				
				//debug printing!
				cout << "Raycast collision: " << col_raycast << endl;		
				cout << "m_col_obj1: " << m_col_obj1 << endl;					
				cout << "m_col_obj1: " << m_col_obj1 << endl;					
				
				m_pDebugDrawer->drawSphere(RayCallback.m_hitPointWorld[0], 3.0, btVector3(1.0f, 1.7f, 1.0f));			
			}
		}
		
		
		State state = State(trans_pack.getOrigin().getX(), trans_pack.getOrigin().getZ(), v_pack[0], v_pack[2],
				trans_robot.getOrigin().getX(), trans_robot.getOrigin().getZ(), v_robot[0], v_robot[2],
				m_col_obj1, m_col_obj1);	
				
		m_robot->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
		m_robot_command = m_robot_strategy->simulator_state(state);			
		m_change_robot_vel = true;		
		m_col_obj1 = "no_object";
		m_col_obj1 = "no_object";				
	}
}

