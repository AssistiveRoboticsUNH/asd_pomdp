 
 /*****************************************************/
 /* Programmer: Victoria Albanese                     */
 /* Program Name: print.cc                            */
 /* Program Description: This program attempts to     */
 /*                      print the output of the NAO's*/
 /*                      tactile touch sensors to an  */
 /*                      output file, output.txt      */
 /*****************************************************/
 

 /*****************************************************/

 // Include Statements
 #include <ros/ros.h>
 #include <iostream>
 #include <fstream>
 #include <ctime>
 #include <naoqi_bridge_msgs/TactileTouch.h>

 /*****************************************************/

 // Namespace Declarations
 using namespace std;
 
 /*****************************************************/

 // Define Statements

 #define FRONT_BUTTON = 1,
 #define MIDDLE_BUTTON = 2,
 #define BACK_BUTTON = 3

 #define BUTTON_PUSHED = 0,
 #define BUTTON_RELEASED = 1
 
 /*****************************************************/

 // Type Definitions
 
 typedef struct button {
    int name;
    bool state;
    bool changed;
 } BUTTON;
 
 /*****************************************************/

 // Global Variables
 
 BUTTON button;

 /*****************************************************/

 // Function Headers
 
 void cb_t_touch(const naoqi_bridge_msgs::TactileTouch::ConstPtr& head_button);
 
 /*****************************************************/

 // Main 
 
 int main(int argc, char **argv) {

    /*****************************************************/
 
     // Various Initializations

     // Initializing 'print' topic and node
     ros::init(argc, argv, "print");
     ros::NodeHandle node;

     //All the subscribers
     ros::Subscriber sub_t_touch = node.subscribe("/nao_robot/tactile_touch", 100, cb_t_touch);

    /*****************************************************/
        
     // Open file to write to
     ofstream fout;
     fout.open("src/nao_project_momotaz/print_to_file/output/output.txt");

    /*****************************************************/
     
     // THIS CODE TAKEN FROM STACK OVERFLOW TO WRITE DOWN DATE AND TIME THAT PROGRAM WAS RUN
   
     time_t rawtime;
     struct tm * timeinfo;
     char buffer[80];

     time (&rawtime);
     timeinfo = localtime(&rawtime);

     strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
     std::string str(buffer);

     cout << "\n\n" << str;

     // END CODE TAKEN FROM STACK OVERFLOW

 
    /*****************************************************/

     // Loop rate and while loop are initialized

     cout << "\nProgram Running\n\n";
     fout << "\nProgram Running\n\n";

     ros::Rate loop_rate(15); 
     while (ros::ok()) {  
  
        /*****************************************************/

         // Check to see if button state has changed

         ros::spinOnce();
         /*
         cout << "\n\nButton [ " << button.name << " ] \n";
         cout << "State [ " << button.state << " ] \n";
         cout << "Changed [ " << button.changed << " ] \n";

         getchar();
         */

         if ( button.changed != 0 ) {

             ros::spinOnce();

            /*****************************************************/
       
             // Start of switch statement
             
             switch (button.name) {
     
            /*****************************************************/
             
             case 1:
    
                if ( button.state == 0 ) {
                    cout << "The FRONT BUTTON has been RELEASED\n";
                    fout << "The FRONT BUTTON has been RELEASED\n"; 
                    ros::spinOnce();
               }
    
                else if ( button.state == 1 ) {
                    cout << "The FRONT BUTTON has been PUSHED\n";
                    fout << "The FRONT BUTTON has been PUSHED\n";
                    ros::spinOnce();
               }
    
                else {
                    cout << "Error: unknown button state\n";
                    fout << "Error: unknown button state\n";
                    ros::spinOnce();
               }
        
                break;
         
            /*****************************************************/
                 
             case 2:
        
                if ( button.state == 0 ) {
                    cout << "The MIDDLE BUTTON has been RELEASED\n";
                    fout << "The MIDDLE BUTTON has been RELEASED\n";
                    ros::spinOnce();
               }
    
                else if ( button.state == 1 ) {
                    cout << "The MIDDLE BUTTON has been PUSHED\n";
                    fout << "The MIDDLE BUTTON has been PUSHED\n";
                    ros::spinOnce();
               }
        
                else {
                    cout << "Error: unknown button state\n";
                    fout << "Error: unknown button state\n";
                    ros::spinOnce();
               }
    
                break;
     
            /*****************************************************/
             
             case 3:
    
                if ( button.state == 0 ) {
                    cout << "The BACK BUTTON has been RELEASED\n";
                    fout << "The BACK BUTTON has been RELEASED\n";
                    ros::spinOnce();
               }
    
                else if ( button.state == 1 ) {
                    cout << "The BACK BUTTON has been PUSHED\n";
                    fout << "The BACK BUTTON has been PUSHED\n";
                    ros::spinOnce();
               }
    
                else {
                    cout << "Error: unknown button state\n";
                    fout << "Error: unknown button state\n";
                    ros::spinOnce();
               }
    
                break;
     
            /*****************************************************/
             
             default:
    
               cout << "Error: unknown button name\n";
               fout << "Error: unknown button name\n";
               ros::spinOnce();
  
               break;
     
            /*****************************************************/
            
            }


            // End of switch statement
 
            /*****************************************************/

         }
 
         else {
            
             cout << "No change\n";
             fout << "No change\n";
            
         }
        
        // End of if else statement

        /*****************************************************/
   
     }
    
     cout << "\nProgram Terminated\n\n";
     fout << "\nProgram Terminated\n\n";

     // End of While Loop

   /*****************************************************/

    // Close file to write to
    fout.close();

    /*****************************************************/

    return 0;

 }

 // End of Main
 
 /*****************************************************/

 // Callback Function for Tactile Touch
 
 void cb_t_touch(const naoqi_bridge_msgs::TactileTouch::ConstPtr& head_button) {

//    static int counter = 0;

//    if ( counter == 0 ) {
//        button.name = head_button->button;
//        button.state = head_button->state;
//        button.prev_state = button.state;
//        counter++;
//    }

//    else {
        button.name = head_button->button;

        if ( button.state != head_button->state ) {
            button.changed = 1;
        }

        else {
            button.changed = 0;
        }

        button.state = head_button->state;
//    }

    cout << "Callback function was called.\n";

 }
 
 /*****************************************************/

