/**
  A class implementing a link in a robot manipulator with a single included joint
  Author: Benned Hedegaard
*/

#include "kinematics/link.h"

//
// Default parameter constructor for the Link class
//
Link::Link( const dh_parameters_t& dhArg, const Joint::JointType& jointTypeArg ) :
    joint( 0.0, jointTypeArg ), params( dhArg ){
  switch( jointTypeArg ){
    case Joint::JointType::Revolute:
      joint.value = params.theta;
      break;
    case Joint::JointType::Prismatic:
      joint.value = params.d;
      break;
    default:
      throw std::runtime_error( "[Link Class Error] Unexpected joint type in constructor" );
  }
  update(); // Set the Link's local transformation according to its DH parameters
}

//
// Updates the link transformation based on the preceding Joint's state
//
void Link::update( void ){
  switch( joint.type ){
    case Joint::JointType::Revolute:
      transformation = HomogeneousTransformation::fromDH( params.a, params.alpha, params.d, joint.value );
      return;
    case Joint::JointType::Prismatic:
      transformation = HomogeneousTransformation::fromDH( params.a, params.alpha, joint.value, params.theta );
      return;
    default: // We do not expect any other joint types; throw an error
      throw std::runtime_error( "[Link Class Error] Unexpected joint type in update()" );
  }
}
