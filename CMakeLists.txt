cmake_minimum_required(VERSION 3.5)

project(ahr_sim)
set (BULLET_INCLUDE_DIR "/usr/include/bullet/")
set (CMAKE_CXX_FLAGS "-std=c++14")
set (BULLET_LIBRARIES 
	BulletDynamics 	
	BulletCollision 	
	LinearMath)

find_package(Bullet REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(	
	${BULLET_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}
)
add_executable(ahr_sim	
	AhrSimApplication.cpp
	AhrSimCore.cpp
	DebugDrawer.cpp
	GameObject.cpp
	Strategy.cpp
	main.cpp
)
target_link_libraries(ahr_sim 
	${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY} 	
	${BULLET_LIBRARIES}
	)
