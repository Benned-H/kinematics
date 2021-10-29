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
// Update the stored transformations in each Link of the Manipulator using the current joint variables
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
// Return the end-effector pose for the Manipulator
//
HomogeneousTransformation Manipulator::end_pose( void )const{
  return frames.back();
}

//
// Return the end-effector position (in Frame 0) for the Manipulator
//
Vector3D Manipulator::pe( void )const{
  return frames.back().position();
}

//
// Return the geometric Jacobian column for the given Link (1 to n)
//
Manipulator::J_column_t Manipulator::get_J_column( const int i )const{
  // Check that the requested link exists
  if( i < 1 || i > links.size() ) throw std::runtime_error( "[Manipulator Error] Invalid link requested in get_J_column" );

  // Compute the Jacobian column according to Siciliano Eq. 3.30
  Vector3D z_im1 = frames[i-1].z_axis();
  Manipulator::J_column_t Jcol;

  // Result depends on the Link's Joint type...
  switch( links[i-1].joint.type ){
    case Joint::JointType::Revolute: {
      Vector3D p_im1 = frames[i-1].position();
      Jcol.Pi = z_im1.cross( pe() - p_im1 );
      Jcol.Oi = z_im1;
      break;
    }
    case Joint::JointType::Prismatic: {
      Jcol.Pi = z_im1;
      Jcol.Oi = Vector3D( 0.0, 0.0, 0.0 );
      break;
    }
    default: // We do not expect any other joint types; throw an error
      throw std::runtime_error( "[Manipulator Class Error] Unexpected joint type" );
  }

  return Jcol;
}

//
// Return the 6 x n geometric Jacobian for the entire manipulator
//
Eigen::MatrixXd Manipulator::J( void )const{
  // Iterate over all Links in the Manipulator, computing the Jacobian column for each
  std::vector< Manipulator::J_column_t > J_columns;
  for( int i = 1; i <= links.size(); i++ ){
    J_columns.emplace_back( get_J_column( i ) );
  }

  // Construct and return the 6 x n result
  Eigen::MatrixXd J;
  J.resize( 6, links.size() );
  for( int c = 0; c < J_columns.size(); c++ ){
    J_column_t col = J_columns[c];
    J(0,c) = col.Pi.x();
    J(1,c) = col.Pi.y();
    J(2,c) = col.Pi.z();
    J(3,c) = col.Oi.x();
    J(4,c) = col.Oi.y();
    J(5,c) = col.Oi.z();
  }

  return J;
}

//
// Perturb the joint variables of the Manipulator by the given q_dot values
//
void Manipulator::perturb( const Eigen::MatrixXd& q_dot ){
  if( q_dot.rows() != links.size() ) throw std::runtime_error( "[Manipulator Error] q_dot does not have same length as links in perturb()" );

  // We can loop over the rows of q_dot and update each link accordingly
  for( int i = 0; i < q_dot.rows(); i++ ){
    links[i].joint.value += q_dot(i,0);
  }

  // Update the arm's transformations based on new joint angles
  update();
  return;
}
