/**
  Demo for the Manipulator class
  Author: Benned Hedegaard
*/

#include <iostream>
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "core/homogeneous-transformation.h"
#include "kinematics/manipulator.h"

int main( int argc, char* argv[] ){
  std::cout << "Start of Manipulator class demo program" << std::endl;

  // First, create the links of our 6-DOF arm using their DH parameters
  // Parameters from Siciliano Table 2.6 (anthropomorphic arm with spherical wrist)
  double a2 = 0.5;
  double d4 = 0.8;
  double d6 = 0.3;

  Link::dh_parameters_t dh1 = { 0.0, M_PI/2.0, 0.0, 0.0 };
  Link::dh_parameters_t dh2 = { a2, 0.0, 0.0, 0.0 };
  Link::dh_parameters_t dh3 = { 0.0, M_PI/2.0, 0.0, 0.0 };
  Link::dh_parameters_t dh4 = { 0.0, -M_PI/2.0, d4, 0.0 };
  Link::dh_parameters_t dh5 = { 0.0, M_PI/2.0, 0.0, 0.0 };
  Link::dh_parameters_t dh6 = { 0.0, 0.0, d6, 0.0 };

  // Create vector for our links and populate using the above values
  std::vector< Link > links;
  links.emplace_back( dh1, Joint::JointType::Revolute );
  links.emplace_back( dh2, Joint::JointType::Revolute );
  links.emplace_back( dh3, Joint::JointType::Revolute );
  links.emplace_back( dh4, Joint::JointType::Revolute );
  links.emplace_back( dh5, Joint::JointType::Revolute );
  links.emplace_back( dh6, Joint::JointType::Revolute );

  // Construct the Manipulator object, which will update its transformations automatically
  Manipulator arm(  HomogeneousTransformation::identity(), links );

  std::cout << "Initial end-effector pose: " << arm.end_pose() << std::endl;

  // Set up ROS node and publish arm's transforms
  ros::init(argc, argv, "manipulator_demo");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;
  
  ros::Rate rate( 10.0 );
  while ( ros::ok() ){
    
    std::stringstream frame_name;
    frame_name.str( "frame0" );

    // Each Link contains its own local transformation, already computed
    for( int l = 0; l < arm.links.size(); l++ ){
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = ros::Time::now();
      ts.header.frame_id = frame_name.str();

      // Update the frame name to the current frame's
      frame_name.str("");
      frame_name << "frame" << (l + 1);
      ts.child_frame_id = frame_name.str();
      std::cout << "Broadcasting transform from " << ts.header.frame_id << " to " << ts.child_frame_id << "..." << std::endl;

      ts.transform = arm.links[l].transformation.to_Transform();
      br.sendTransform( ts );
    }
 
    rate.sleep();
    printf("sending\n");
  }

  std::cout << "End of Manipulator class demo program, spinning..." << std::endl;
  return EXIT_SUCCESS;
};
