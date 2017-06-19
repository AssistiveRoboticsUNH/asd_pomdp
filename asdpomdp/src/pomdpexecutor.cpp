/*
pomdpexecutor.cpp
Madison Clark-Turner
1/24/2017
*/
#include "../include/asdpomdp/pomdpexecutor.h"

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

	//setup ROS variables
	r = new ros::Rate(30);
	sub_gaze = n.subscribe("/asdpomdp/gaze", 100, &POMDPExecutor::gazeCallback, this);
	sub_words = n.subscribe("/asdpomdp/audio", 100, &POMDPExecutor::audioCallback, this);
	sub_gesture = n.subscribe("/asdpomdp/gesture", 100, &POMDPExecutor::gestureCallback, this);
	sub_run = n.subscribe("/asdpomdp/run_asd_auto", 100, &POMDPExecutor::runCallback, this);

	pub_nextAct = n.advertise<std_msgs::Int8>("/asdpomdp/next_action", 100);

	//setup POMDP and policy
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
	//begin a session of the behavioral intervention 
	ROS_INFO("Beginning behavioral intervention.");
	callAction(0);
	int nextact = updatePOMDP(0);
	while(nextact >= 0 && nextact < 2){
		nextact = updatePOMDP(nextact);
	}
	if(nextact == 2){
		ros::Duration(startTime[nextact]).sleep();
		callAction(3);
	}
	ROS_INFO("Ending behavioral intervention.");
	policy->reset();
}

void POMDPExecutor::callAction(int action){
	//executes an action
	std_msgs::Int8 performAction;
	performAction.data = action;
	pub_nextAct.publish(performAction);
	ROS_INFO("executed action: %d", action);
}

int POMDPExecutor::updatePOMDP(int action){
	// Listen for observations and update the POMDP.
	if(action >= 0 && action < 3){
		std::vector<int> obs = getObservations(action);
		if(obs.size() == 0){
			return -1;
		}

		// Update Policy
		int obsIndex = convertToIndex(obs);
		printf("Received observation: %d\n", obsIndex);
		policy->printBelief();
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

	// Delay from action start to beginning of listening period.
	while(ros::ok() && ros::Time::now() < startRecord){
		ros::spinOnce();
	}
	// Reset observation information prior to listening period.
	averageGazeDistance = 0;
	numberOfPacketsReceived = 0;
	audioObserved = 0;
	gestureObserved = 0;

	while(ros::ok() && ros::Time::now() < endRecord){
		// Gather observtaion information during listening period.
		ros::spinOnce();
	}
	if(!ros::ok()){
		return obs;
	}
	
	//get whether gaze is directed at robot
	if(numberOfPacketsReceived > 0)
		obs.push_back((averageGazeDistance/numberOfPacketsReceived) < threshold);
	else
		obs.push_back(0);
	
	//get whether audio was observed
	obs.push_back(audioObserved);
	
	//get whether a gesture was observed.
	obs.push_back(gestureObserved);

	ROS_INFO("OBSERVATION: gaze-%d audio-%d gesture-%d", obs[0], obs[1], obs[2]);
	return obs;
}

void POMDPExecutor::gazeCallback(const std_msgs::Float64::ConstPtr& msg){
	averageGazeDistance += msg->data;
	numberOfPacketsReceived++;
}

void POMDPExecutor::audioCallback(const std_msgs::Bool::ConstPtr& msg){
	if(msg->data)
		audioObserved = 1;
}

void POMDPExecutor::gestureCallback(const std_msgs::String::ConstPtr& msg){
	gestureObserved = 1;
}

void POMDPExecutor::runCallback(const std_msgs::Bool::ConstPtr& msg){
	run();
}
