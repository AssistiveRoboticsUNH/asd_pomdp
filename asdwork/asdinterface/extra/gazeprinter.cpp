#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <std_msgs/String.h>
//#include <algorithm>
//#include <geometry_msgs/Twist.h>

bool gazeFocusedOnRobot(tf::StampedTransform transform, double range){
	//ROS_INFO("pos: (%f, %f, %f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
	tf::Quaternion rot = transform.getRotation();
	ROS_INFO("angle: (%f, %f, %f) %f", rot.getAxis().x(), rot.getAxis().y(), rot.getAxis().z(), range);
	ROS_INFO("true: %d, false: %d", true, false);
	ROS_INFO("true: %f, false: %f", std::abs(rot.getAxis().x()), std::abs(rot.getAxis().z()));
	if(std::abs(rot.getAxis().x()) < range && std::abs(rot.getAxis().z()) < range){
		return true;
	}
	return false;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "gaze_printer");
    ros::NodeHandle n;
    ros::Rate r(30);

	tf::TransformListener listener;
    std::vector<std::string> frames;

    ros::Publisher pub_speak = n.advertise<std_msgs::String>("/gaze", 100); //publisher to make nao talk

	ROS_INFO("Waiting until a face becomes visible...");

	while (ros::ok())
	{
		while (!listener.waitForTransform("base_footprint", "face_0", ros::Time::now(), ros::Duration(5.0))) {
		    ROS_INFO("Still no face visible...");
		    r.sleep();
		}
		//ROS_INFO("Face is visible...%lu frames", frames.size());
		/*
		frames.clear();
		listener.getFrameStrings(frames);
		//listener.lookupTransform(observer_frame, target_frame, ros::Time(0), transform);
		std::string s;
		std_msgs::String words;
		for(int i = 0; i < frames.size(); i++){
			//ROS_INFO("frame %d: %s", i, frames[i].c_str());
			if(frames[i].find("face_0") == 0)
				s = frames[i];
		}
		ROS_INFO("Head Angles: %s",s.c_str());
		words.data = "\\RSPD=70\\Head Angles: ";
		pub_speak.publish(words);
		*/

		tf::StampedTransform transform;

	    try{
	        listener.lookupTransform("face_0", "CameraTop_optical_frame", ros::Time(0), transform);
	        bool focused = gazeFocusedOnRobot(transform, 0.5);
	        ROS_INFO("facingRobot: %d", focused);
	        
	    }
	    catch (tf::TransformException ex){
	        ROS_ERROR("%s",ex.what());
	        ros::Duration(1.0).sleep();
	    }



			
		r.sleep();
	}
}