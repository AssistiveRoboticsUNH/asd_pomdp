/* Author: Madison Clark-Turner */

#ifndef AUDIO_LOGGER_H_
#define AUDIO_LOGGER_H_

#include <ros/ros.h>
#include <nao_interaction_msgs/AudioRecorder.h>
#include <string>
#include <custom_msgs/control_states.h>
#include <std_msgs/Bool.h>
#include <iostream>

class AudioLogger
{
public:
  AudioLogger(ros::NodeHandle);
  ~AudioLogger();

  void cb(const custom_msgs::control_states States);
  std::string getTimeStamp();
  void rec(const std_msgs::Bool is_recording);


private:

	ros::NodeHandle n;

	std::string bag_path_;
  ros::ServiceClient client_record;
	ros::Subscriber sub_control, sub_recording;

	nao_interaction_msgs::AudioRecorder record;

	custom_msgs::control_states states;


};


#endif /* AUDIO_LOGGER_H_ */

