 
 /*****************************************************/
 /* Programmer: Victoria Albanese                     */
 /* Program Name: rename.cpp                          */
 /* Program Description: This program moves a bag     */
 /*                      file from the ~/.ros/        */
 /*                      directory to the local folder*/
 /*                      data/bag_files               */
 /*****************************************************/
 

 /*****************************************************/

    // Include Statements
 
    #include <stdio.h>
    #include <iostream>
    #include <string>
    #include <stdlib.h>
    #include <dirent.h>
    #include <regex>
    #include <ros/ros.h>
    #include <custom_msgs/control_states.h>

 /*****************************************************/

    // Define Statements
  
    #define MAX_DATE 12

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
        ros::init(argc, argv, "get_timestamp");
        ros::NodeHandle node;
 
        // Initializing the subscriber object
        ros::Subscriber sub_ctrl = node.subscribe("control_msgs", 100, controlcb);

        // Declare a variable
        // custom_msgs::control_states timestamp;

        /*****************************************************/

        // Variable declarations
 
        string current_directory_path;
        const char * current_path;

        string new_directory_path;
        const char * new_path;        
        
        int num_files;
        string file_name = "session_";
        string file_extension = ".bag";
 
        string current_file_name;
        const char * current_name;
        string new_file_name;
        const char * new_name;

        /*****************************************************/

        // Initialize current and new paths of rosbag
          
        current_directory_path = getenv("HOME");
        current_directory_path+= "/bag/";
        current_path = current_directory_path.c_str();

        new_directory_path = getenv("HOME");
        new_directory_path+= "/bag/";
        new_path = new_directory_path.c_str();

        /*****************************************************/
   
        // Get the name of the bag file
        
        ros::spinOnce();
        while ( start_clicked.startrecord == false ) {
            ros::spinOnce();
            ROS_INFO("Waiting for timestamp...");
        } 
        
	for(int i = 0; i < 5; i++){
		ros::spinOnce();
	}

        file_name+= start_clicked.timestamp;

        /*****************************************************/
    
        // Wait for shutdown to move the file
        
        ros::spinOnce();
        while ( start_clicked.shutdown == false ) {
            ros::spinOnce();
            ROS_INFO("Waiting for shutdown...");
        } 
        
        
        /*****************************************************/

        // Change the path of the rosbag
                
        current_file_name = current_directory_path + file_name + file_extension;
        new_file_name = new_directory_path + file_name + file_extension;

        cout << "\nRenaming bag file " << current_file_name << " at " << current_directory_path << "...\n\n";

        current_name = current_file_name.c_str();
        new_name = new_file_name.c_str();

        rename( current_name, new_name );
        
        /*****************************************************/

        return 0;

    }

    // End of Main
 
 /*****************************************************/
 

