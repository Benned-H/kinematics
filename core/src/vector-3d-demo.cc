/**
  Demo for the Vector3D class
  Author: Benned Hedegaard
*/

#include <iostream>
#include "core/vector-3d.h"

int main( int argc, char* argv[] ){
  std::cout << "Start of Vector3D class demo program" << std::endl;

  // Test empty and parameter constructors
  Vector3D empty_vector = Vector3D();
  std::cout << "Empty vector:" << empty_vector << std::endl;

  Vector3D default_vector = Vector3D( 1.0, 2.0, 3.0 );
  std::cout << "Vector3D(1,2,3):" << default_vector << std::endl;

  // Test negation
  std::cout << default_vector << " negated is " << ( -default_vector ) << std::endl;

  // Test to and from Eigen::Vector3d functions
  Eigen::Vector3d eigen_vector( 4.0, 6.0, 8.0 );
  Vector3D from_eigen = Vector3D( eigen_vector );
  Eigen::Vector3d eigen_result = from_eigen.to_Vector3d();
  std::cout << "Eigen::Vector3d " << eigen_vector << " cast to Vector3D:" << from_eigen << std::endl;
  std::cout << "Eigen::Vector3d after casting back:" << eigen_result << std::endl;

  // Test in-place addition and subtraction
  Vector3D one_vector = Vector3D( 1.0, 1.0, 1.0 );
  default_vector += one_vector;
  std::cout << "After adding " << one_vector << ":" << default_vector << std::endl;

  default_vector += one_vector;
  std::cout << "After adding " << one_vector << " again:" << default_vector << std::endl;

  Vector3D two_vector = Vector3D( 2.0, 2.0, 2.0 );
  default_vector -= two_vector;
  std::cout << "After subtracting " << two_vector << ":" << default_vector << std::endl;

  // Test norm method
  Vector3D v100 = Vector3D( 1.0, 0.0, 0.0 );
  Vector3D v010 = Vector3D( 0.0, 1.0, 0.0 );
  Vector3D v001 = Vector3D( 0.0, 0.0, 1.0 );

  std::cout << "Norm of " << default_vector << " is " << default_vector.norm() << std::endl;
  std::cout << "Norm of " << one_vector << " is " << one_vector.norm() << std::endl;
  std::cout << "Norm of " << two_vector << " is " << two_vector.norm() << std::endl;
  std::cout << "Norm of " << v100 << " is " << v100.norm() << std::endl;
  std::cout << "Norm of " << v010 << " is " << v010.norm() << std::endl;
  std::cout << "Norm of " << v001 << " is " << v001.norm() << std::endl;

  // Test set and x/y/z getter functions
  default_vector.set( 5.0, 10.0, 20.0 );
  std::cout << "Default vector was set to [5,10,20] and is now:" << default_vector << std::endl;
  std::cout << "x is " << default_vector.x() << ", y is " << default_vector.y() << ", and z is " << default_vector.z() << std::endl;

  // Test addition and subtraction
  std::cout << v100 << " + " << v010 << " = " << ( v100 + v010 ) << std::endl;
  std::cout << v010 << " + " << v001 << " = " << ( v010 + v001 ) << std::endl;
  std::cout << default_vector << " - " << v001 << " = " << ( default_vector - v001 ) << std::endl;
  std::cout << v010 << " - " << default_vector << " = " << ( v010 - default_vector ) << std::endl;

  std::cout << "End of Vector3D class demo program" << std::endl;
  return EXIT_SUCCESS;
}
