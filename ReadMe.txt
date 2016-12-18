Automated ASD Therapy Robot
==========================================
Madison Clark-Turner   (12/17/2016)
==========================================
The following code includes the ROS implementation that allows for the automated 
execution of an ABA greeting therapy. The code uses a POMDP to model the 
interaction with humans and collects observations using a NAO humanoid robot's
microphone and main (top) camera.

The implementation is intended for use with the following ROS packages:

ROS NAO - https://github.com/ros-naoqi/nao_robot
attention-tracking - https://github.com/chili-epfl/attention-tracker

Having installed the other packages find and replace the 'nao_full_py.launch' file 
in the rospackage nao_bringup with the file of the same name included in the 
directory. This file activates various NAO apps including nao_alife, nao_speech, 
nao_behaviors, and nao_audio.

This package also includes two directories: 

asdwork - code by Alexander Infantino, an interface for teleoperating the NAO robot.
	This code possesses significant changes from its source material with regards
	to the interface design and functionality. The code relating to the wave 
	functionality is unchanged.
asdpomdp - the automated model used to deliver the therapy. The majority of work
	I performed is here.

To execute the program you must execute the following code, each in its own 
terminal window:

roslaunch nao_bringup nao_full_py.launch
roslaunch asdpomdp asdpomdp_auto.launch

After starting the NAO robot's speech recognition system the robot may begin 
moving their limbs and placing them in its hips. This functionality should be
disabled by using

rosrun asdwork turnoffautonomousmoves.py

Once the interface has displayed select the 'Start button'. The NAO will go 
into a resting position and will stand and tilt his head forward showing 
that he is listening. To begin the therapy speak to the robot and clearly 
state: "Ready to Begin".

- - - - - - - - - - - - - - - - - - - - - - - 

Here we provide a file breakdown for each package:

ASDWORK:
	asdinterface/include/asdinterface/asdinterface.hpp - class header, includes 
		button functionality and name of user. The name of the user must be
		updated and the files re-compiled for each new user
	asdinterface/include/asdinterface/ui_interface.hpp - Qt interface design
	asdinterface/src/asdinterface.cpp - button implementation, changes from
		the original include disabling autonomous life, tilting of head, 
		designated start and stop record functions, stand and rest buttons. 
		The interface subscribes to rostopic information about actions to 
		perform and publishes information about when actions complete.

ASDPOMDP:
	launch/asdpomdp_auto.launch - launches the automated ABA therapy
	launch/asdpomdp.launch - launches the teleoperated therapy with record 
		functionality
	src/asd.pomdp - POMDP model of therapy
	src/asd.vector - alpha vector POMDP policy 
	src/audio.cpp - rosnode that activates and listens to audio responses	
	src/gaze.cpp - rosnode that receives tf messages from the attention-tracking
		package and publishes messages when the distance is within a threshold	
	src/main.cpp - executes the automated policy.
	src/pomdp.h	- file that reads a .pomdp file
	src/pomdp.cpp - file that reads a .pomdp file
	src/pomdppolicy.h - file that reads a .vector file and updates a POMDP policy
	src/pomdppolicy.cpp - file that reads a .vector file and updates a POMDP policy
	src/pomdpexecutor.h - rosnode that executes the ABA therapy
	src/pomdpexecutor.cpp - rosnode that executes the ABA therapy
	src/turnoffautonomousmoves.py - turns off autonomous life