/**
  Demo for the RotationMatrix class
  Author: Benned Hedegaard
*/

#include <iostream>
#include "core/rotation-matrix.h"

int main( int argc, char* argv[] ){
  std::cout << "Start of RotationMatrix class demo program" << std::endl;

  // Test both parameter constructors and identity constructor
  Vector3D v100 = Vector3D( 1.0, 0.0, 0.0 );
  Vector3D v010 = Vector3D( 0.0, 1.0, 0.0 );
  Vector3D v001 = Vector3D( 0.0, 0.0, 1.0 );
  RotationMatrix default_matrix = RotationMatrix( v100, v010, v001 );
  std::cout << "Default matrix constructed:" << default_matrix << std::endl;

  Eigen::Matrix3d R_x_up;
  R_x_up << 0, 0, -1,
            0, 1, 0,
            1, 0, 0;
  RotationMatrix x_up_matrix = RotationMatrix( R_x_up );
  std::cout << "Matrix with x-axis up constructed:" << x_up_matrix << std::endl;
  std::cout << "Identity rotation matrix constructor:" << RotationMatrix::identity() << std::endl;

  // Test element access function
  std::cout << "Element (0,0) of " << default_matrix << ":" << default_matrix(0,0) << std::endl;
  std::cout << "Element (0,2) of " << x_up_matrix << ":" << x_up_matrix(0,2) << std::endl;

  // Test set and x/y/z getter functions
  Vector3D y_up_x_axis = Vector3D( 1.0, 0.0, 0.0 );
  Vector3D y_up_y_axis = Vector3D( 0.0, 0.0, 1.0 );
  Vector3D y_up_z_axis = Vector3D( 0.0, -1.0, 0.0 );
  default_matrix.set( y_up_x_axis, y_up_y_axis, y_up_z_axis );
  std::cout << "Matrix set to have y-axis up:" << default_matrix << std::endl;
  std::cout << "x axis:" << default_matrix.x_axis() << std::endl;
  std::cout << "y axis:" << default_matrix.y_axis() << std::endl;
  std::cout << "z axis:" << default_matrix.z_axis() << std::endl;

  // Test rotation matrix multiplication
  RotationMatrix I = RotationMatrix::identity();
  std::cout << I << " * " << default_matrix << " = " << ( I * default_matrix ) << std::endl;
  std::cout << default_matrix << " * " << I << " = " << ( default_matrix * I ) << std::endl;
  std::cout << default_matrix << " * " << x_up_matrix << " = " << ( default_matrix * x_up_matrix ) << std::endl;
  std::cout << x_up_matrix << " * " << default_matrix << " = " << ( x_up_matrix * default_matrix ) << std::endl;

  // Test multiplication with Vector3D
  std::cout << I << " * " << y_up_z_axis << " = " << ( I * y_up_z_axis ) << std::endl;
  Vector3D v123 = Vector3D( 1.0, 2.0, 3.0 );
  std::cout << default_matrix << " * " << v123 << " = " << ( default_matrix * v123 ) << std::endl;

  // Test inverse function
  RotationMatrix I_inverse = I.inverse();
  RotationMatrix default_inverse = default_matrix.inverse();
  RotationMatrix x_up_inverse = x_up_matrix.inverse();

  std::cout << I << " multiplied by its inverse " << I_inverse << " is " << ( I * I_inverse ) << std::endl;
  std::cout << default_matrix << " multiplied by its inverse " << default_inverse << " is " << ( default_matrix * default_inverse ) << std::endl;
  std::cout << x_up_matrix << " multiplied by its inverse " << x_up_inverse << " is " << ( x_up_matrix * x_up_inverse ) << std::endl;
  
  std::cout << "End of RotationMatrix class demo program" << std::endl;
  return EXIT_SUCCESS;
}
