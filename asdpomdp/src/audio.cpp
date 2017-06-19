/*
audio.cpp
Madison Clark-Turner
11/2/2016
///////////////////////////////////
This node listens to the output of the NAO's word recognizer and returns a bool describing
whether a word was heard or not. It begins by initalizing the NAO's vocabulary with a
vector of strings that NAO can identify.

**autonomous life needs to be disabled before running this program or vocabularies from other
applications could be idenitifed by NAO's word recognizer

*/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <nao_msgs/SetSpeechVocabularyAction.h>

#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nao_msgs/WordRecognized.h"

#include <iostream>
#include <string>
#include <vector>

bool wordIdentified;
ros::Publisher pub_speak, pub_run;

void wordRecognized(const nao_msgs::WordRecognized::ConstPtr& msg){
	// returns true if NAO recognizes a word, false otherwise
	wordIdentified = true;
	std_msgs::Bool obsReceived;
	obsReceived.data = wordIdentified;
	if(msg->words[0] == "ready to begin"){
		pub_run.publish(obsReceived);
	}else if(msg->words[0] != "great job" and msg->words[0] != "good bye"){
		pub_speak.publish(obsReceived);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "audio_observer");
	ros::NodeHandle n;
	ros::Rate r(30);

	// action required to set the NAO's vocabulary
	actionlib::SimpleActionClient<nao_msgs::SetSpeechVocabularyAction> set_vocab("speech_vocabulary_action", true);

	// rosservice that starts the recognizer
	ros::ServiceClient stop_recognizer = n.serviceClient<std_srvs::Empty>("/stop_recognition", 100);
	ros::ServiceClient start_recognizer = n.serviceClient<std_srvs::Empty>("/start_recognition", 100);
	ros::ServiceClient disable_expressive_listening = n.serviceClient<std_srvs::Empty>("/disable_expressive_listening");

	// rostopic contingency will be published to
	ros::Subscriber sub_words = n.subscribe<nao_msgs::WordRecognized>("/word_recognized", 1000, wordRecognized);
	pub_speak = n.advertise<std_msgs::Bool>("asdpomdp/audio", 1000);
	pub_run = n.advertise<std_msgs::Bool>("/asdpomdp/run_asd_auto", 1000);

	ROS_DEBUG( "Wait on action servers..." );
	set_vocab.waitForServer();
	ROS_DEBUG( "Connected!" );

	// Send Vocab to NAO
	nao_msgs::SetSpeechVocabularyGoal config;
	std::vector<std::string> s {"hello", "hi", "good morning", "hi there", "no", "ready to begin", "great job", "good bye"};
	config.words = s;
	set_vocab.sendGoal(config);
	
	// Start Recognizer
	set_vocab.waitForResult();
	std_srvs::Empty srv;
	
	stop_recognizer.call(srv);
	start_recognizer.call(srv);

	if (disable_expressive_listening.call(srv))
	{
		ROS_INFO("Successfully called service disable_expressive_listening");
	}
	else
	{
		ROS_ERROR("Failed to call service disable_expressive_listening");
	}
	
	// Start publishing if words recognized
	ros::spin();
}
