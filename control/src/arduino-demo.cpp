/**
  Demo for testing the Arduino Mega interface for servo control
  Author: Benned Hedegaard
*/

#include <iostream>
#include "ros/ros.h"
#include "control/JointCommand.h"

int main( int argc, char* argv[] ){
  std::cout << "Start of Arduino control demo program" << std::endl;

  // Set up ROS node and publish some example servo configurations
  ros::init(argc, argv, "arduino_demo");
  ros::NodeHandle nh;

  ros::Publisher command_pub = nh.advertise< control::JointCommand >( "control/commands", 1, true );

  // Configuration 1 - All servos with angle 0
  control::JointCommand command;
  command.angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  command_pub.publish( command );
  
  ros::Rate timer(1); // Timer at 1Hz
  timer.sleep(); // Wait 1 second

  // Configuaration 2 - Alternating angles 0 and 90
  command.angles = {0.0, 90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
  command_pub.publish( command );

  // Configuration 3 - All servos with inverse angles (0 or 90) as in Config 3
  command.angles = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0, 90.0};
  command_pub.publish( command );

  std::cout << "End of Arduino control demo program, spinning..." << std::endl;
  return EXIT_SUCCESS;
};
