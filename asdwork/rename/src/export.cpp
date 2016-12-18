 
 /*****************************************************/
 /* Programmer: Victoria Albanese                     */
 /* Program Name: export.cpp                          */
 /* Program Description: This program calls the python*/
 /*                      file which exports the images*/
 /*                      from the rosbag video        */
 /*                      recording                    */
 /*****************************************************/
 

 /*****************************************************/

    // Include Statements
 
    #include <stdio.h>
    #include <iostream>
    #include <string>
    #include <stdlib.h>
    #include <dirent.h>
    #include <ros/ros.h>
    #include <ros/package.h>
    #include <custom_msgs/control_states.h>

 /*****************************************************/

    // Namespace Declarations

    using namespace std;
 
 /*****************************************************/

    // Declare global variable
    custom_msgs::control_states start_clicked;

    // Declare callback function to get
    void controlcb(const custom_msgs::control_states what_was_clicked){
            start_clicked = what_was_clicked;
    }
 
 /*****************************************************/

    // Main 
 
    int main(int argc, char **argv) {

        /*****************************************************/

        // Ros declarations

        // Initializing "get_timestamp" topic and node
        ros::init(argc, argv, "export");
        ros::NodeHandle node;
 
        // Initializing the subscriber object
        ros::Subscriber sub_ctrl = node.subscribe("control_msgs", 100, controlcb);

        // Declare a variable
        // custom_msgs::control_states timestamp;

        /*****************************************************/
        
        // Variable declarations
        string timestamp;
        string package_path;
        string file_name;
        string arg0;
        string arg1;
        string arg2;
        string call_export;
   
        /*****************************************************/
            
        // Get the timestamp of the start time
          
        ros::spinOnce();
        while ( start_clicked.startrecord == false ) {
            ros::spinOnce();	
        }
 
        timestamp = start_clicked.timestamp;
	ROS_INFO_STREAM("TIMESTAMP IS: " << timestamp);

	while(start_clicked.shutdown == false){
		ros::spinOnce();
		//ROS_INFO("WAITING FOR SHUTDOWN");
	}
		
        /*****************************************************/

        // Variable definitions
        package_path = ros::package::getPath("rename");
        file_name = "session_" + timestamp + ".bag";
        arg0 = package_path + "/src/export.py";
        arg1 = getenv("HOME");
        arg1+= "/bag/" + file_name + " ";
        arg2 = "/nao_robot/camera/top/camera/image_raw";
    
        call_export = "python " + arg0 + " " + arg1 + " " + arg2;

        /*****************************************************/

        // Make a syscall to run export.py 
   
        const char * command = call_export.c_str();
   
        if ( !system(command) ) {
            cout << "Export syscall failed" << endl;
        }
        
        /*****************************************************/
   
        return 0;

    }

    // End of Main
 
 /*****************************************************/


