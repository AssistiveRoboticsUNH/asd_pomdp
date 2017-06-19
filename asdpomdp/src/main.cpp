/*
main.cpp
Madison Clark-Turner
1/24/2017
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <iostream>
#include "../include/asdpomdp/pomdpexecutor.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "asd_pomdp");
	ros::NodeHandle n;
	srand(time(0));
	std::string path = ros::package::getPath("asdpomdp")+'/';
	POMDPExecutor p(n, path+"src/asd.pomdp", path+"src/asd.vector");
	ROS_INFO("POMDP Executor ready");
	ros::spin();
}
