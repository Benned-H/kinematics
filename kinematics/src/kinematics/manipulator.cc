/**
  A class implementing a robot manipulator composed of rigid-body links forming a kinematic chain
  Author: Benned Hedegaard
*/

#include "kinematics/manipulator.h"

//
// Default parameter constructor for the Manipulator class
//
Manipulator::Manipulator( const HomogeneousTransformation& baseFrameArg,
                          const std::vector<Link>& linksArg ) : links( linksArg ), frames(){
  // First push Frame 0 onto the frames vector
  frames.emplace_back( baseFrameArg );
  
  // Then update all link tranformations along with the manipulator's frames
  update();
}

//
// Updates the stored transformations in each Link of the Manipulator using the current joint variables
//
void Manipulator::update( void ){
  // Exit if the manipulator has no links
  if( links.empty() ) return;
  
  // Update the transformation per link
  for( Link& link : links ){
    link.update();
  }

  // Clear the frames vector except the base frame
  frames.erase( frames.begin() + 1, frames.end() );

  // Compute Frames 1 to n for the manipulator
  for( Link& link : links ){
    frames.emplace_back( frames.back() * link.transformation );
  }
  
  return;
}

//
// Returns the end-effector pose for the Manipulator
//
HomogeneousTransformation Manipulator::end_pose( void )const{
  return frames.back();
}
