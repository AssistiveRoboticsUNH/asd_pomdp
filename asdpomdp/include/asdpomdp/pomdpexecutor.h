/*
pomdpexecutor.h
Madison Clark-Turner
1/24/2017
*/
#ifndef POMDPExecutor_H
#define POMDPExecutor_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <iostream>
#include <cmath>
#include "pomdp.h"
#include "pomdppolicy.h"

class POMDPExecutor{
private:
	//POMDPExecutor information
	POMDP* pomdp;
	POMDPPolicy * policy;
	std::random_device rd;

	//non-const POMDPExecutor information 
	int curState;
	double threshold = 0.243;

	double averageGazeDistance;
	unsigned numberOfPacketsReceived;
	int audioObserved;
	int gestureObserved;

	int* startTime;// number of seconds after action has concluded to start looking for contingency
	int* endTime;// number of seconds after starting to look for contingency that we stop #[7,7,2.197]

	// ROS topics
	ros::NodeHandle n;
	ros::Rate* r;
	ros::Subscriber sub_gaze, sub_words, sub_gesture, sub_run;
	ros::Publisher pub_nextAct;

	// callbacks
	void gazeCallback(const std_msgs::Float64::ConstPtr&);
	void audioCallback(const std_msgs::Bool::ConstPtr&);
	void gestureCallback(const std_msgs::String::ConstPtr&);
	void runCallback(const std_msgs::Bool::ConstPtr& msg);

	std::vector<int> getObservations(int);
	int convertToIndex(std::vector<int>);

	int updatePOMDP(int); // accepts action, returns observation
	void callAction(int action);

public:
	POMDPExecutor(ros::NodeHandle, std::string, std::string);
	~POMDPExecutor();

	std::string getCurrentState(){return pomdp->states[curState];}
	void run();
};

#endif
