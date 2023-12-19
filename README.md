### AhrSimulator
Air hockey robot simulator is a project that has been inspired by an idea to educate my older son
to create interesting robot projects. He is 12 year old and I was looking for a fun project that 
combines robotics and software development.

After examination a number of projects I decided to stick with open source air hockey robot project - https://github.com/JJulio/AHRobot.	
This project provideds a brilliant instructions how to build air hockey robot.
Robot's parts were printed by 3D pronter and all other stuff was found in the nearest store. 
So a few days latter my son and me have managed to assemble the robot and replicate the game construction. 
The project uses web camera and simple openCV tracking algorithm in order to steer the robot. 
They also provide all required software to start playing the game.
So we are really excited by playing the game: https://drive.google.com/open?id=1Jig2CLq_RQDYSkCrVZRSssE4OTvT7oh2

However performance of the robot was disappointing.  
Even my younger son who is 6 year old was able to win the game easily. 
So I decided to take the project to the next stage and significantly improve the robot performance.
My idea was to use reinforce learning algorithm in order to train the robot play game and eventually create a stronger game starategy. 

Reinforcement learning (RL) is a subfield of machine learning that involves training an agent to make a series of decisions in an environment in order to maximize a reward.
The goal of RL is to learn a policy that maps states to actions in a way that maximizes the expected cumulative reward over time.

In order to train RL algorithm I needed a robot simulator because training involves a playing a big number of games in order to find the best policy.
After a research I did not find appropriate simulator for air hockey robot and I decided to develop my own simulator.
This repository contains my implementation of the air hockey robot simulator for training RL algorightms.
The technology that has been chosen is Bullet Physics SDK (https://github.com/bulletphysics/bullet3)
for real world forces simulation and OpenGL for visualization. 

#
Main point for strategy is abstract Strategy class. All RL policies must be inherited from this class.
In order to show how air hockey simulator works I implemented simply policy ToyStrategy that plays the game:
https://drive.google.com/open?id=1aRo8Zmy9rUqJ-_tABO-29AAIfiIhBQo2

#
Next step is to start training advanced strategies by RL algorithm and the simulator.
TODO: define reward function for the next step policy
#

### Requirements for AhrSimulator
The simulator was developed and tested on Ubunutu 16.04 with C++, Bullet ver 2.87,
nvidia-396 OpenGL driver and Glut library.

