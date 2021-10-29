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

  // Initialize gain parameters for position and orientation
  double kP = 0.1;
  double kO = 0.1;

  // Create an example desired pose for the end-effector
  Vector3D pd = Vector3D( 0.3, 0.3, 0.8 );
  std::cout << "Desired end-effector position:" << pd << std::endl;

  Eigen::Matrix3d Rd_m;
  Rd_m << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;
  RotationMatrix Rd = RotationMatrix( Rd_m );
  geometry_msgs::Quaternion qd = Rd.to_Quaternion();
  double eta_d = qd.w;
  Vector3D eps_d = Vector3D( qd.x, qd.y, qd.z );

  std::cout << "Desired end-effector orientation:" << Rd << std::endl;

  // Set up ROS node and publish arm's transforms
  ros::init(argc, argv, "manipulator_demo");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;
  
  // ros::Rate rate( 2 );
  while ( ros::ok() ){
    
    // Goal: Compute the state update q_dot based on current Manipulator state
    // 1. Position error can be computed using the current and desired end-effector position
    Vector3D eP = pd - arm.pe();
    std::cout << "Current position error:" << eP << std::endl;

    // 2. Orientation error requires the current and desired end-effector rotation as a quaternion
    geometry_msgs::Quaternion qe = arm.end_pose().orientation().to_Quaternion();
    double eta_e = qe.w;
    Vector3D eps_e = Vector3D( qe.x, qe.y, qe.z );
    Vector3D eO = (eta_e * eps_d) - (eta_d * eps_e) - (S( eps_d ) * eps_e);
    std::cout << "Current orientation error:" << eO << std::endl;

    // 3. Put position and orientation errors together in a 6 x 1 result
    // Assumes desired end-effector linear and angular velocity are both zero
    Eigen::MatrixXd M_error; // 6 x 1 matrix containing error terms of product
    M_error.resize( 6, 1 );
    M_error << kP * eP.x(),
               kP * eP.y(),
               kP * eP.z(),
               kO * eO.x(),
               kO * eO.y(),
               kO * eO.z();
    std::cout << "Error matrix part of product:" << std::endl;
    std::cout << M_error << std::endl;

    // 4. Compute current manipulator geometric Jacobian
    Eigen::MatrixXd J = arm.J();
    std::cout << "Current geometric Jacobian:" << std::endl;
    std::cout << J << std::endl;

    Eigen::MatrixXd J_inv = J.completeOrthogonalDecomposition().pseudoInverse();
    std::cout << "J_inv is:" << std::endl;
    std::cout << J_inv << std::endl;

    Eigen::MatrixXd q_dot = J_inv * M_error;
    std::cout << "Current q_dot is:" << std::endl;
    std::cout << q_dot << std::endl << std::endl;

    // Update the arm's pose based on q_dot
    arm.perturb( q_dot );

    /*std::cout << "Waiting for duration of 1 Hz loop..." << std::endl;
    rate.sleep();*/
    
    // Publish the state of the arm! Each Link contains its own local transformation, already computed
    std::stringstream frame_name;
    frame_name.str( "frame0" );

    std::vector< geometry_msgs::TransformStamped > transforms;
    for( int l = 0; l < arm.links.size(); l++ ){
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = ros::Time::now();
      ts.header.frame_id = frame_name.str();

      // Update the frame name to the current frame's
      frame_name.str("");
      frame_name << "frame" << (l + 1);
      ts.child_frame_id = frame_name.str();

      ts.transform = arm.links[l].transformation.to_Transform();
      transforms.push_back( ts );
    }

    std::cout << "Broadcasting transforms..." << std::endl;
    br.sendTransform( transforms );

    std::cout << "Waiting for user input..." << std::endl;
    std::cin.get();
 }

  std::cout << "End of Manipulator class demo program, spinning..." << std::endl;
  return EXIT_SUCCESS;
};
