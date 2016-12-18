#include <ros/ros.h>
#include <string>
#include <custom_msgs/control_states.h>
#include <iostream>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/BodyPoseActionGoal.h>


custom_msgs::control_states states;

void cb(const custom_msgs::control_states States){
	states = States;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "nao_wave");
	ros::NodeHandle n;

	ros::Subscriber sub_control = n.subscribe("/control_msgs", 100, cb);
	ros::Publisher pub_control = n.advertise<custom_msgs::control_states>("/control_msgs", 100);
	ros::Publisher pub_move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);
	ros::Publisher pub_pose = n.advertise<naoqi_bridge_msgs::BodyPoseActionGoal>("/body_pose/goal", 100);

	ros::Rate loop_rate(15);

	nao_msgs::JointAnglesWithSpeed hy, hr, ler, ley, lh, lsp, lsr, lwy, rer,rey, rh, rsp, rsr, rwy;

	hy.joint_names.push_back("HeadYaw");
	hr.joint_names.push_back("HeadRoll");
	ler.joint_names.push_back("LElbowRoll");
	ley.joint_names.push_back("LElbowYaw");
	lh.joint_names.push_back("LHand");
	lsp.joint_names.push_back("LShoulderPitch");
	lsr.joint_names.push_back("LShoulderRoll");
	lwy.joint_names.push_back("LWristYaw");
	rer.joint_names.push_back("RElbowRoll");
	rey.joint_names.push_back("RElbowYaw");
	rh.joint_names.push_back("RHand");
	rsp.joint_names.push_back("RShoulderPitch");
	rsr.joint_names.push_back("RShoulderRoll");
	rwy.joint_names.push_back("RWristYaw");
	hy.joint_angles.push_back(0);
	hr.joint_angles.push_back(0);
	ler.joint_angles.push_back(0);
	ley.joint_angles.push_back(0);
	lh.joint_angles.push_back(0);
	lsp.joint_angles.push_back(0);
	lsr.joint_angles.push_back(0);
	lwy.joint_angles.push_back(0);
	rer.joint_angles.push_back(0);
	rey.joint_angles.push_back(0);
	rh.joint_angles.push_back(0);
	rsp.joint_angles.push_back(0);
	rsr.joint_angles.push_back(0);
	rwy.joint_angles.push_back(0);

	naoqi_bridge_msgs::BodyPoseActionGoal pose;

	while(ros::ok()){
		ros::spinOnce();
		if(states.startwave1 == false && states.startwave2 == false && states.shutdown == false){
			ros::spinOnce();
		}
		else if(states.shutdown == true){
			ROS_INFO("SHUTTING DOWN WAVE");
			ros::shutdown();
		}
		else if(states.startwave1 == true){
		
			ROS_INFO("WAVING1");
			ros::Duration(.64).sleep();
				
			// .64
			rer.joint_angles[0] = 1.3852;
			rer.speed = 0.10667;
			rey.joint_angles[0] = -0.3129;
			rey.speed = 0.2533;
			rsp.joint_angles[0] = 0.2470;
			rsp.speed = 0.2533;
			rsr.joint_angles[0] = -0.242;
			rsr.speed = 0.2533;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rsp);
			pub_move.publish(rsr);		
			ros::Duration(.12).sleep();

			// .72
			ler.joint_angles[0] = -1.37902;
			ler.speed = 0.2667;
			ley.joint_angles[0] = -0.803;
			ley.speed = 0.2667;
			lsp.joint_angles[0] = 1.118;
			lsp.speed = 0.2667;
			lsr.joint_angles[0] = 0.3635;
			lsr.speed = 0.2667;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			ros::Duration(.68).sleep();
	
			//1.4
			rer.joint_angles[0] = 0.2424;
			rer.speed = .1066;
			rey.joint_angles[0] = 0.5644;
			rey.speed = 0.2667;
			rh.joint_angles[0] = 0.0;
			rh.speed = 0.1;
			rsp.joint_angles[0] = -1.1719;
			rsp.speed = 0.2266;
			rsr.joint_angles[0] = -0.9541;
			rsr.speed = 0.2667;
			rwy.joint_angles[0] = -0.3129;
			rwy.speed = 0.1;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rh);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			pub_move.publish(rwy);
			ros::Duration(.12).sleep();
	
			//1.52
			ler.joint_angles[0] = -1.29005;
			ler.speed = 0.2667;
			ley.joint_angles[0] = -0.6918;
			ley.speed = 0.2667;
			lh.joint_angles[0] = 0;
			lh.speed = 0.1;
			lsp.joint_angles[0] = 0.928;
			lsp.speed = 0.2667;
			lsr.joint_angles[0] = 0.2269;
			lsr.speed = .2667;
			lwy.joint_angles[0] = 0.147;
			lwy.speed = 0.1;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lh);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			pub_move.publish(lwy);
			ros::Duration(.2).sleep();
		
			//1.72
			rer.joint_angles[0] = 0.349;
			rer.speed = 0.12;
			pub_move.publish(rer);
			ros::Duration(.36).sleep();

			//2.08
			rer.joint_angles[0] = 0.9342;
			rer.speed = 0.1;
			rey.joint_angles[0] = 0.39112;
			rey.speed = 0.1;
			rsp.joint_angles[0] = -1.089;
			rsp.speed = 0.1;
			rsr.joint_angles[0] = -0.460;
			rsr.speed = 0.1;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			ros::Duration(.12).sleep();

			//2.2
			ler.joint_angles[0] = -1.1827;
			ler.speed = 0.2667;
			ley.joint_angles[0] = -0.679;
			ley.speed = 0.2667;
			lsp.joint_angles[0] = 0.9403;
			lsp.speed = .2667;
			lsr.joint_angles[0] = 0.2038;
			lsr.speed = 0.2667;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			ros::Duration(.4).sleep();			

			pose.goal.pose_name = "Stand";
			pub_pose.publish(pose);

			// at end publish false so does not loop
			states.startwave1 = false;
			pub_control.publish(states);
		}
		else if(states.startwave2 == true){

			ROS_INFO("WAVE 2");
			ros::Duration(.12).sleep();

			//.12
			rer.joint_angles[0] = 0.6806;
			rer.speed = 0.0933;
			pub_move.publish(rer);
			ros::Duration(.28).sleep();

			//.4
			rer.joint_angles[0] = 0.1919;
			rer.speed = 0.1333;
			rey.joint_angles[0] = 0.3481;
			rey.speed = 0.24;
			rsp.joint_angles[0] = -1.260;
			rsp.speed = 0.24;
			rsr.joint_angles[0] = -0.960;
			rsr.speed = 0.24;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			ros::Duration(.12).sleep();

			//.56
			ler.joint_angles[0] = -1.248;
			ler.speed = 0.2667;
			ley.joint_angles[0] = -0.6105;
			ley.speed = 0.2667;
			lsp.joint_angles[0] = 0.862;
			lsp.speed = 0.2667;
			lsr.joint_angles[0] = 0.2177;
			lsr.speed = 0.2667;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lsp);
			pub_move.publish(lsr);

			//.8
			rer.joint_angles[0] = 0.2617;
			rer.speed = 0.1066;
			pub_move.publish(rer);
			ros::Duration(.32).sleep();
			
			//1.12
			rer.joint_angles[0 ] = 0.7072;
			rer.speed = 0.1466;
			rey.joint_angles[0] = 0.3819;
			rey.speed = 0.1466;
			rh.joint_angles[0] = 0.01;
			rh.speed = 0.3733;
			rsp.joint_angles[0] = -1.1489;
			rsp.speed = 0.3733;
			rsr.joint_angles[0] = -0.3283;
			rsr.speed = 0.3733;
			rwy.joint_angles[0] = -0.3037;
			rwy.speed = 0.3733;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rh);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			pub_move.publish(rwy);
			ros::Duration(.12).sleep();

			//1.24
			ler.joint_angles[0] = -1.319;
			ler.speed = 0.36;
			ley.joint_angles[0] = -0.753;
			ley.speed = 0.36;
			lsp.joint_angles[0] = 0.897;
			lsp.speed = 0.36;
			lsr.joint_angles[0] = 0.2484;
			lsr.speed = 0.36;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			ros::Duration(.32).sleep();

			//1.56
			rer.joint_angles[0] = 1.0192;
			rer.speed = 0.2266;
			rey.joint_angles[0] = 0.977;
			rey.speed = 0.2266;
			pub_move.publish(rer);
			pub_move.publish(rey);
			ros::Duration(.68).sleep();

			//2.24
			rer.joint_angles[0] = 1.2655;
			rer.speed = 0.0266;
			rey.joint_angles[0] = 0.826;
			rey.speed = 0.0266;
			rh.joint_angles[0] = 0;
			rh.speed = 1;
			rsp.joint_angles[0] = 1.02015;
			rsp.speed = 0.0266;
			rsr.joint_angles[0] = -0.25;
			rsr.speed = 0.0266;
			rwy.joint_angles[0] = 0.1825;
			rwy.speed = 0.0266;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rh);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			pub_move.publish(rwy);
			ros::Duration(.08).sleep();
		
			//2.32
			ler.joint_angles[0] = -1.184;
			ler.speed = 0.16;
			ley.joint_angles[0] = -0.67;
			ley.speed = 0.16;
			lh.joint_angles[0] = 0;
			lh.speed = 0.16;
			lsp.joint_angles[0] = 0.842;
			lsp.speed = 0.16;
			lsr.joint_angles[0] = 0.2269;
			lsr.speed = 0.16;
			lwy.joint_angles[0] = 0.1196;
			lwy.speed = 0.16;
			rer.joint_angles[0] = 1.219;
			rer.speed = 0.16;
			rey.joint_angles[0] = 0.847;
			rey.speed = 0.16;
			rsp.joint_angles[0] = 1.135;
			rsp.speed = 0.16;
			rsr.joint_angles[0] = -0.2369;
			rsr.speed = 0.16;
			rwy.joint_angles[0] = 0.1779;
			rwy.speed = 0.16;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lh);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			pub_move.publish(lwy);
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			pub_move.publish(rwy);
			ros::Duration(.48).sleep();

			//2.8
			ler.joint_angles[0] = -0.4239;
			ler.speed = 0.16;
			ley.joint_angles[0] = -1.198;
			ley.speed = 0.16;
			lsp.joint_angles[0] = 1.45;
			lsp.speed = 0.16;
			lsr.joint_angles[0] = 0.14;
			lsr.speed = 0.16;
			lwy.joint_angles[0] = 0.0998;
			lwy.speed = 0.16;
			rer.joint_angles[0] = 0.424;
			rer.speed = 0.16;
			rey.joint_angles[0] = 1.1975;
			rey.speed = 0.16;
			rsp.joint_angles[0] = 1.45;
			rsp.speed = 0.16;
			rsr.joint_angles[0] = -0.135;
			rsr.speed = 0.16;
			rwy.joint_angles[0] = 0.09;
			rwy.speed = 0.16;
			pub_move.publish(ler);
                        pub_move.publish(ley);
                        pub_move.publish(lh);
                        pub_move.publish(lsp);
                        pub_move.publish(lsr);
                        pub_move.publish(lwy);
                        pub_move.publish(rer);
                        pub_move.publish(rey);
                        pub_move.publish(rsp);
                        pub_move.publish(rsr);
                        pub_move.publish(rwy);
			ros::Duration(.4).sleep();

			//1.24
			ler.joint_angles[0] = -1.319;
			ler.speed = 0.36;
			ley.joint_angles[0] = -0.753;
			ley.speed = 0.36;
			lsp.joint_angles[0] = 0.897;
			lsp.speed = 0.36;
			lsr.joint_angles[0] = 0.2484;
			lsr.speed = 0.36;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			ros::Duration(.32).sleep();

			//1.56
			rer.joint_angles[0] = 1.0192;
			rer.speed = 0.2266;
			rey.joint_angles[0] = 0.977;
			rey.speed = 0.2266;
			pub_move.publish(rer);
			pub_move.publish(rey);
			ros::Duration(.68).sleep();

			//2.24
			rer.joint_angles[0] = 1.2655;
			rer.speed = 0.0266;
			rey.joint_angles[0] = 0.826;
			rey.speed = 0.0266;
			rh.joint_angles[0] = 0;
			rh.speed = 1;
			rsp.joint_angles[0] = 1.02015;
			rsp.speed = 0.0266;
			rsr.joint_angles[0] = -0.25;
			rsr.speed = 0.0266;
			rwy.joint_angles[0] = 0.1825;
			rwy.speed = 0.0266;
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rh);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			pub_move.publish(rwy);
			ros::Duration(.08).sleep();
		
			//2.32
			ler.joint_angles[0] = -1.184;
			ler.speed = 0.16;
			ley.joint_angles[0] = -0.67;
			ley.speed = 0.16;
			lh.joint_angles[0] = 0;
			lh.speed = 0.16;
			lsp.joint_angles[0] = 0.842;
			lsp.speed = 0.16;
			lsr.joint_angles[0] = 0.2269;
			lsr.speed = 0.16;
			lwy.joint_angles[0] = 0.1196;
			lwy.speed = 0.16;
			rer.joint_angles[0] = 1.219;
			rer.speed = 0.16;
			rey.joint_angles[0] = 0.847;
			rey.speed = 0.16;
			rsp.joint_angles[0] = 1.135;
			rsp.speed = 0.16;
			rsr.joint_angles[0] = -0.2369;
			rsr.speed = 0.16;
			rwy.joint_angles[0] = 0.1779;
			rwy.speed = 0.16;
			pub_move.publish(ler);
			pub_move.publish(ley);
			pub_move.publish(lh);
			pub_move.publish(lsp);
			pub_move.publish(lsr);
			pub_move.publish(lwy);
			pub_move.publish(rer);
			pub_move.publish(rey);
			pub_move.publish(rsp);
			pub_move.publish(rsr);
			pub_move.publish(rwy);
			ros::Duration(.48).sleep();

			//2.8
			ler.joint_angles[0] = -0.4239;
			ler.speed = 0.16;
			ley.joint_angles[0] = -1.198;
			ley.speed = 0.16;
			lsp.joint_angles[0] = 1.45;
			lsp.speed = 0.16;
			lsr.joint_angles[0] = 0.14;
			lsr.speed = 0.16;
			lwy.joint_angles[0] = 0.0998;
			lwy.speed = 0.16;
			rer.joint_angles[0] = 0.424;
			rer.speed = 0.16;
			rey.joint_angles[0] = 1.1975;
			rey.speed = 0.16;
			rsp.joint_angles[0] = 1.45;
			rsp.speed = 0.16;
			rsr.joint_angles[0] = -0.135;
			rsr.speed = 0.16;
			rwy.joint_angles[0] = 0.09;
			rwy.speed = 0.16;
			pub_move.publish(ler);
                        pub_move.publish(ley);
                        pub_move.publish(lh);
                        pub_move.publish(lsp);
                        pub_move.publish(lsr);
                        pub_move.publish(lwy);
                        pub_move.publish(rer);
                        pub_move.publish(rey);
                        pub_move.publish(rsp);
                        pub_move.publish(rsr);
                        pub_move.publish(rwy);
			ros::Duration(.4).sleep();

			pose.goal.pose_name = "Stand";
			pub_pose.publish(pose);			

			states.startwave2 = false;
			pub_control.publish(states);
		}
	}
	return 0;
}
