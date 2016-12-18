/************************************/
/* Programmer: Alexander Infantino  */
/* Program Name: test_print         */
/* Program Description: The purpose */
/*      of this program is to test  */
/*      to see if I know how to     */
/*      sensor status to files      */
/************************************/

// Include Statements
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Bool.h>
#include <ctime>

using namespace std;

// Global Variables
bool footContact = true;

void footCB(const std_msgs::Bool footBool){
    footContact = footBool.data;
}

int main(int argc, char ** argv){

    // Initializes ROS
    ros::init(argc, argv, "print_foot_status");
    ros::NodeHandle node;

    // Subscribes to foot_contact topic
    ros::Subscriber sub_foot = node.subscribe("/foot_contact", 100, footCB);

    // Opens file feet_contact_output.txt to write to
    std::ofstream fout;
    fout.open("src/nao_project_momotaz/print_to_file/output/feet_contact_log.txt", std::ofstream::out | std::ofstream::app);

    // THIS CODE TAKEN FROM STACK OVERFLOW TO WRITE DOWN DATE AND TIME THAT PROGRAM WAS RUN
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
    std::string str(buffer);
    // END CODE TAKEN FROM STACK OVERFLOW
   
    fout << "PROGRAM RAN: " << str << "\n\n";

    while(ros::ok()){
        ros::spinOnce();
        std::cout << "Determining feet contact\n";
        ros::Duration(2).sleep();
        ros::spinOnce();

        if(footContact == true){
            std::cout << "Feet are touching the ground!\n";
            fout << "--- Feet Touching Ground == TRUE ---\n\n";
            ros::Duration(2).sleep();
            ros::spinOnce();
        }

        else if(footContact == false){
            std::cout << "Feet are off of the ground!\n";
            fout << "--- Feet Touching Ground == FALSE ---\n\n";
            ros::Duration(2).sleep();
            ros::spinOnce();
        }
        else{
            std::cout << "UNKNOWN ERROR\n";
            fout << "--- UNKNOWN ERROR ---\n\n";
            ros::Duration(2).sleep();
            ros::spinOnce();
        }
    }

        fout << "PROGRAM TERMINATED\n\n\n\n";

        fout.close();

    return 0;
}
