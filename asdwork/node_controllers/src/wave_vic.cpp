#include <ros/ros.h>
#include <string>
#include <custom_msgs/control_states.h>
#include <iostream>
#include <nao_msgs/JointAnglesWithSpeed.h>

custom_msgs::control_states states;

void cb(const custom_msgs::control_states States){
	states = States;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "nao_record_microphone");
	ros::NodeHandle n;

	ros::Subscriber sub_control = n.subscribe("/control_msgs", 100, cb);
	ros::Publisher pub_control = n.advertise<custom_msgs::control_states>("/control_msgs", 100);
	ros::Publisher move = n.advertise<nao_msgs::JointAnglesWithSpeed>("joint_angles", 100);

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

	while(ros::ok()){
		ros::spinOnce();
		if(states.startwave1 == false){
			ros::spinOnce();
		}
		else if(states.shutdown == true){
			ROS_INFO("SHUTTING DOWN WAVE");
			ros::shutdown();
		}
		else if(states.startwave1 == true){
	
            /*****************************************/

            // clump of right arm movememnt 1
            ros::Duration(0.64).sleep();

            rer.joint_angles[0] = 1.38;
            rer.speed = 0.21;
            move.publish(ler);
	
            rey.joint_angles[0] = -0.31;
            rey.speed = 0.21;
            move.publish(ley);
				
	        rsp.joint_angles[0] = 0.24;
            rsp.speed = 0.21;
            move.publish(lsp);
				
	        rsr.joint_angles[0] = -0.24;
            rsr.speed = 0.21;
            move.publish(lsr);
										
            /*****************************************/

            // clump of left arm movememnt 1
            ros::Duration(0.08).sleep();

            ler.joint_angles[0] = -1.37;
            ler.speed = 0.24;
            move.publish(ler);
	
            ley.joint_angles[0] = -0.80;
            ley.speed = 0.24;
            move.publish(ley);
				
	        lsp.joint_angles[0] = 1.11;
            lsp.speed = 0.24;
            move.publish(lsp);
				
	        lsr.joint_angles[0] = 0.36;
            lsr.speed = 0.24;
            move.publish(lsr);

            /*****************************************/
	
            // clump of right arm movement 2
            ros::Duration(0.68).sleep();
	
            rer.joint_angles[0] = 0.24;
            rer.speed = 0.25;
            move.publish(rer);
				
            rey.joint_angles[0] = 0.56;
            rey.speed = 0.25;
            move.publish(rey);
				
            rh.joint_angles[0] = 0.014;
            rh.speed = 0.46;
            move.publish(rh);
	
            rsp.joint_angles[0] = -1.17;
            rsp.speed = 0.25;
            move.publish(rsp);
	        
            rsr.joint_angles[0] = -0.95;
            rsr.speed = 0.25;
            move.publish(rsr);
	
            rwy.joint_angles[0] = -0.31;
            rwy.speed = 0.46;
            move.publish(rwy);
			
            /*****************************************/

            // clump of left arm movement 2
            ros::Duration(0.12).sleep();
	
            ler.joint_angles[0] = -1.29;
            ler.speed = 0.26;
            move.publish(ler);
				
            ley.joint_angles[0] = -0.69;
            ley.speed = 0.26;
            move.publish(ley);
				
            lh.joint_angles[0] = 0.004;
            lh.speed = 0.50;
            move.publish(lh);
	
            lsp.joint_angles[0] = 0.92;
            lsp.speed = 0.26;
            move.publish(lsp);
	        
            lsr.joint_angles[0] = 0.22;
            lsr.speed = 0.26;
            move.publish(lsr);
	
            lwy.joint_angles[0] = 0.14;
            lwy.speed = 0.50;
            move.publish(lwy);

            /*****************************************/
			
            // clump of right arm movement 3
            ros::Duration(0.2).sleep();
			
            rer.joint_angles[0] = 0.34;
            rer.speed = 0.10;
            move.publish(rer);
	
            rey.joint_angles[0] = -0.67;
            rey.speed = 0.22;
            move.publish(rey);
	
            /*****************************************/

            // clump of right arm movement 4
            ros::Duration(0.35).sleep();
			
            rer.joint_angles[0] = 0.93;
            rer.speed = 0.12;
            move.publish(rer);
	
            rey.joint_angles[0] = 0.39;
            rey.speed = 0.22;
            move.publish(rey);
						
	        rsp.joint_angles[0] = -1.08;
            rsp.speed = 0.22;
            move.publish(rsp);
						
	        rsr.joint_angles[0] = -0.46;
            rsr.speed = 0.22;
            move.publish(rsr);

            /*****************************************/

            // clump of left arm movement 3
            ros::Duration(0.12).sleep();
			
            ler.joint_angles[0] = -1.18;
            ler.speed = 0.22;
            move.publish(ler);
	
            ley.joint_angles[0] = -0.67;
            ley.speed = 0.22;
            move.publish(ley);
						
	        lsp.joint_angles[0] = 0.94;
            lsp.speed = 0.22;
            move.publish(lsp);
						
	        lsr.joint_angles[0] = 0.20;
            lsr.speed = 0.22;
            move.publish(lsr);
	
            /*****************************************/

			// at end publish false so does not loop
			states.startwave1 = false;
			pub_control.publish(states);
		}
	}
	return 0;
}
