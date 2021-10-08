/**
  Class to represent a joint, either revolute or prismatic, in a kinematic chain
  Author: Benned Hedegaard
*/

#include "kinematics/joint.h"

//
// Default parameter constructor for the Joint class
//
Joint::Joint( const double valueArg, const JointType typeArg ) : value( valueArg ), type( typeArg ){

}
