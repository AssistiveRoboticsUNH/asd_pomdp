/*
gaze.cpp
Madison Clark-Turner
1/24/2017
///////////////////////////////////
This node listens to the output of the attention-tracker package. If the package broadcasts a tf message
then the message is queried to identify whether the gaze is within a specific range. If it is a 'true'
bool is published to the /gaze_contingency rostopic.
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <cmath>

double rad2deg(double inp){
	//convert radians to degrees
	return inp * 3.141592653589793 / 180.0;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "gaze_observer");
	ros::NodeHandle n;
	ros::Rate r(30);

	tf::TransformListener listener;
	std::vector<std::string> frames;

	ros::Publisher pub_data = n.advertise<std_msgs::Float64>("/asdpomdp/gaze", 100);

	ROS_DEBUG("Waiting until a face becomes visible...");

	while (ros::ok())
	{
		while (!listener.waitForTransform("base_footprint", "face_0", ros::Time::now(), ros::Duration(5.0))) {
			ROS_DEBUG("No face visible...");
			r.sleep();
		}
		
		tf::StampedTransform transform;

		try{
			//extract relevant info from tf
			listener.lookupTransform("face_0", "CameraTop_optical_frame", ros::Time(0), transform);

			//calculate distance of gaze to robot
			tf::Quaternion rot = transform.getRotation();
			tf::Vector3 pos = transform.getOrigin();
			double focalpointx = std::tan(rad2deg(rot.getAxis().x())) * pos.getX() + (-1*pos.getZ())-0.5;
			double focalpointy = std::tan(rad2deg(rot.getAxis().z())) * pos.getX() + pos.getY()-0.5;
			double distfrompoint = std::abs(std::sqrt(pow(focalpointx, 2)+pow(focalpointy, 2)));

			//publish distance of gaze from robot
			std_msgs::Float64 data;
			data.data = distfrompoint;
			pub_data.publish(data);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		r.sleep();
	}
}
