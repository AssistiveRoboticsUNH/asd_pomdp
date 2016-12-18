#include <ros/ros.h>
#include <nao_interaction_msgs/AudioRecorder.h>
#include <string>
#include <custom_msgs/control_states.h>
#include <iostream>

custom_msgs::control_states states;

void cb(const custom_msgs::control_states States){
	states = States;
}

std::string getTimeStamp(){
	time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        //strftime(buffer,80,"%d-%m-%Y %I:%M:%S", timeinfo);
        strftime(buffer,80,"%Y-%m-%d-%I-%M-%S", timeinfo);
        std::string str(buffer);
        return str;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "ros_bag_record");
	ros::NodeHandle n;
	static int count = 1;

	// subscribe to custom msgs to start up
	ros::Subscriber sub_control = n.subscribe("/control_msgs", 100, cb);

	nao_interaction_msgs::AudioRecorder record;

	while(ros::ok()){
		ros::spinOnce();
		if(states.startbag == false){
			ros::spinOnce();
		}
		else if(states.shutdown == true){
			ROS_INFO("SHUTTING DOWN ROSBAG RECORDER");
			ros::shutdown();
		}
		else if(states.startbag == true){
			if(count == 1){
				std::string timestamp;
				timestamp = states.timestamp;
				ROS_INFO("RECORDING BAG");
				system("rosbag record /nao_robot/camera/top/camera/image_raw");
				ROS_INFO("DONE RECORDING BAG");
				count++;
			}
		}
	}
	return 0;
}
