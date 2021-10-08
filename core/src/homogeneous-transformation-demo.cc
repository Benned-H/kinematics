/**
  Demo for the HomogeneousTransformation class
  Author: Benned Hedegaard
*/

#include <iostream>
#include "core/homogeneous-transformation.h"

int main( int argc, char* argv[] ){
  std::cout << "Start of HomogeneousTransformation class demo program" << std::endl;

  // Test identity transformation and parameter constructors
  HomogeneousTransformation I = HomogeneousTransformation::identity();
  std::cout << "Constructed identity HomogeneousTransformation:" << I << std::endl;

  Vector3D v123 = Vector3D( 1.0, 2.0, 3.0 );
  Eigen::Matrix3d R_x_up;
  R_x_up << 0, 0, -1,
            0, 1, 0,
            1, 0, 0;
  RotationMatrix x_up_matrix = RotationMatrix( R_x_up );
  HomogeneousTransformation default_T = HomogeneousTransformation( v123, x_up_matrix );
  std::cout << "Constructed HomogeneousTransformation with position:" << v123 << " and orientation:" << x_up_matrix << std::endl;
  std::cout << "Result is " << default_T << std::endl;
  
  // Test position and orientation get functions
  std::cout << "This has position " << default_T.position() << std::endl;
  std::cout << "and orientation " << default_T.orientation() << std::endl;

  // Test set function
  Vector3D v0 = Vector3D( 0.0, 0.0, 0.0 );
  default_T.set( v0, x_up_matrix );
  std::cout << "Set position to " << v0 << " giving result:" << std::endl;
  std::cout << default_T << std::endl;

  RotationMatrix R_I = RotationMatrix::identity();
  default_T.set( v0, R_I );
  std::cout << "Set orientation to " << R_I << " giving result:" << std::endl;
  std::cout << default_T << std::endl;

  default_T.set( v123, x_up_matrix );

  // Test inverse function
  HomogeneousTransformation I_inv = I.inverse();
  HomogeneousTransformation default_inv = default_T.inverse();
  std::cout << I << " has inverse " << I_inv << std::endl;
  std::cout << default_T << " has inverse " << default_inv << std::endl;

  // Test HomogeneousTransformation multiplication
  std::cout << I << " * " << default_T << " = " << ( I * default_T ) << std::endl;
  std::cout << default_T << " * " << I << " = " << ( default_T * I ) << std::endl;
  std::cout << default_inv << " * " << default_T << " = " << ( default_inv * default_T ) << std::endl;
  std::cout << default_T << " * " << default_inv << " = " << ( default_T * default_inv ) << std::endl;

  std::cout << "End of HomogeneousTransformation class demo program" << std::endl;
  return EXIT_SUCCESS;
}
