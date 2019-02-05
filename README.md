### AhrSimulator
Air hockey robot simulator is a project that has been inspired by an idea to educate my older son
to create interesting robot projects. He is 12 year old and I was looking for some fun project that 
combines robotics and software development.

 After a few projects discovered I decided to stick with open source air hockey robot project - https://github.com/JJulio/AHRobot.	
They provided a brilliant instructions how to replicate their air hockey robot.

Robot parts were printed by 3D and all other stuff was found in the nearest store. 
So a few days latter we managed to assemble the robot and replicate the game construction. 
The project uses web camera and simple openCV tracking algorithm in order to steer the robot. 
They also provide all required software to start playing the game.
So we are really excited by playing the game: https://drive.google.com/open?id=1Jig2CLq_RQDYSkCrVZRSssE4OTvT7oh2


However performance of the robot was a bit disappointing.  
Even my younger son who is 6 year old is able to win the game easily. 
So I decided to take the project to the next stage and significantly improve the robot performance.
My idea is to use reinforce learning algorithms in order to train the robot best strategy. 
In order to apply RF algorithms I need appropriate simulator. 
After short research I did not find appropriate simulator and decided to develop my one.
The technology that has been chosen is Bullet Physics SDK for real world forces simulation and OpenGL for visualization. 

#
This repository contains a basic code for the air hockey game robot simulation.
Main point for strategy is abstract Strategy.cpp class.
This demo contains game playing by robot against manual adversarial managed by ToyStrategy 
class implementing very a simply strategy -https://drive.google.com/open?id=1aRo8Zmy9rUqJ-_tABO-29AAIfiIhBQo2

#
Next step is to start training advanced strategies by RF algorithms and the simulator.
#

### Requirements for AhrSimulator
I developted the simulator on Ubunutu 16.04, C++, Bullet ver 2.87,
nvidia-396 OpenGL driver and Glut library.

