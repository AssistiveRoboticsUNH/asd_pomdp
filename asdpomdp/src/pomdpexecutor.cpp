/*
pomdpexecutor.cpp
Madison Clark-Turner
12/17/2016
*/
#include "pomdpexecutor.h"

POMDPExecutor::POMDPExecutor(ros::NodeHandle node, std::string pomdpFilename, std::string vectorFilename): n(node)
{	
	//set times when listening occurs
	startTime = new int[3];
	startTime[0] = 7;
	startTime[1] = 7;
	startTime[2] = 0;
	endTime = new int[3];
	endTime[0] = startTime[0]+4;
	endTime[1] = startTime[1]+4;
	endTime[2] = startTime[2]+0;


	r = new ros::Rate(30);
	sub_gaze = n.subscribe("/asdpomdp/gaze_data", 100, &POMDPExecutor::gazeCallback, this);
	sub_words = n.subscribe("/asdpomdp/audio", 100, &POMDPExecutor::audioCallback, this);
	sub_gesture = n.subscribe("/asdpomdp/gesture_contingency", 100, &POMDPExecutor::gestureCallback, this);
	sub_run = n.subscribe("/asdpomdp/run_asd_auto", 100, &POMDPExecutor::runCallback, this);

	pub_nextAct = n.advertise<std_msgs::Int8>("/next_action", 100);

	//setup POMDP
	pomdp = new POMDP(pomdpFilename);
	policy = new POMDPPolicy(pomdp, vectorFilename);

	//randomly assign starting state
	curState = (rand()%2);
}

POMDPExecutor::~POMDPExecutor(){
	delete pomdp;
	delete policy;
	delete[] startTime;
	delete[] endTime;
	delete r;
}

int POMDPExecutor::convertToIndex(std::vector<int> obs){
	// Convert a vector of boolean operations to a single observation index value using bitwise operations
	int index = 0;
	for(int o = 0; o < std::sqrt(pomdp->observations); o++){
		index = index << 1;
		index = index | obs[o];
	}
	return index;
}

void POMDPExecutor::run(){
	//begin the therapy session
	callAction(0);
	int nextact = updatePOMDP(0);
	while(nextact >= 0 && nextact < 2){
		nextact = updatePOMDP(nextact);
	}
	if(nextact == 2){
		ros::Duration(startTime[nextact]).sleep();
		callAction(3);
	}
	ROS_INFO("run ended");
	policy->reset();
}

void POMDPExecutor::callAction(int action){
	//execute an action
	std_msgs::Int8 performAction;
	performAction.data = action;
	pub_nextAct.publish(performAction);
	ROS_INFO("action called: %d", action);
}

int POMDPExecutor::updatePOMDP(int action){
	//receive observation
	
	if(action >= 0 && action < 3){
		std::vector<int> obs = getObservations(action);
		if(obs.size() == 0){
			return -1;
		}

		// Update Policy
		int obsIndex = convertToIndex(obs);
		printf("current obs: %d\n", obsIndex);
		policy->updateBeliefState(obsIndex, action);
		policy->printBelief();
		// Suggest next Action
		int nextActionIndex = policy->selectAction();
		callAction(nextActionIndex);
		return nextActionIndex;
	}
	return action;
}

std::vector<int> POMDPExecutor::getObservations(int act){
	std::vector<int> obs;
	ros::Time begin = ros::Time::now();
	ros::Time startRecord = begin + ros::Duration(startTime[act]);
	ros::Time endRecord = begin + ros::Duration(endTime[act]);
	//do not receive observations while performing action
	while(ros::ok() && ros::Time::now() < startRecord){
		ros::spinOnce();
	}
	averageGazeDistance = 0;
	numberOfPacketsReceived = 0;
	audioContingency = 0;
	gestureContingency = 0;
	//begin receving actions
	while(ros::ok() && ros::Time::now() < endRecord){
		ros::spinOnce();
	}
	if(!ros::ok()){
		return obs;
	}
	
	//get whether audio contingency is detected
	obs.push_back(audioContingency);
	//get whether gaze is directed at robot
	if(numberOfPacketsReceived > 0)
		obs.push_back((averageGazeDistance/numberOfPacketsReceived) < threshold);
	else
		obs.push_back(0);
	
	return obs;
}

void POMDPExecutor::gazeCallback(const std_msgs::Float64::ConstPtr& msg){
	averageGazeDistance += msg->data;
	numberOfPacketsReceived++;
}

void POMDPExecutor::audioCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data)
		audioContingency = 1;
}

void POMDPExecutor::gestureCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data)
		gestureContingency = 1;
}

void POMDPExecutor::runCallback(const std_msgs::Bool::ConstPtr& msg){
	run();
}