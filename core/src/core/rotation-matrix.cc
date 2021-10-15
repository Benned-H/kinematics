/**
  A class implementing a 3x3 rotation matrix representing orientation
  Each column of the matrix corresponds to the x, y, and z axis of the relative frame w.r.t. the previous frame

  Author: Benned Hedegaard
*/

#include "core/rotation-matrix.h"

//
// Identity rotation matrix constructor
//
RotationMatrix RotationMatrix::identity( void ){
  return RotationMatrix( Eigen::Matrix< double, 3, 3 >::Identity() );
}

//
// Vector3D parameter constructor for the RotationMatrix class
//
RotationMatrix::RotationMatrix( const Vector3D& x_axis, const Vector3D& y_axis, const Vector3D& z_axis ) : R(3,3){
  set( x_axis, y_axis, z_axis );
}

//
// Matrix3d parameter constructor for the RotationMatrix class
//
RotationMatrix::RotationMatrix( const Eigen::Matrix3d& rArg ) : R( rArg ){

}

//
// RotationMatrix set function
//
void RotationMatrix::set( const Vector3D& x_axis, const Vector3D& y_axis, const Vector3D& z_axis ){
  R(0,0) = x_axis.x();
  R(1,0) = x_axis.y();
  R(2,0) = x_axis.z();
  R(0,1) = y_axis.x();
  R(1,1) = y_axis.y();
  R(2,1) = y_axis.z();
  R(0,2) = z_axis.x();
  R(1,2) = z_axis.y();
  R(2,2) = z_axis.z();
}

//
// RotationMatrix element access function
//
double RotationMatrix::operator()( const int r, const int c )const{
  return R(r,c);
}

//
// Return the x-axis of the RotationMatrix
//
Vector3D RotationMatrix::x_axis( void )const{
  return Vector3D( R(0,0), R(1,0), R(2,0) );
}

//
// Return the y-axis of the RotationMatrix
//
Vector3D RotationMatrix::y_axis( void )const{
  return Vector3D( R(0,1), R(1,1), R(2,1) );
}

//
// Return the z-axis of the RotationMatrix
//
Vector3D RotationMatrix::z_axis( void )const{
  return Vector3D( R(0,2), R(1,2), R(2,2) );
}

//
// Return the inverse of the RotationMatrix
//
RotationMatrix RotationMatrix::inverse( void )const{
  return RotationMatrix( R.transpose() );
}

//
// Return the RotationMatrix as a geometry_msgs::Quaternion
//
geometry_msgs::Quaternion RotationMatrix::to_Quaternion( void )const{
  // See Siciliano Eq. 2.34 and 2.35
  geometry_msgs::Quaternion q;
  q.x = 0.5 * sign( R(2,1) - R(1,2) ) * sqrt( R(0,0) - R(1,1) - R(2,2) + 1.0 );
  q.y = 0.5 * sign( R(0,2) - R(2,0) ) * sqrt( R(1,1) - R(2,2) - R(0,0) + 1.0 );
  q.z = 0.5 * sign( R(1,0) - R(0,1) ) * sqrt( R(2,2) - R(0,0) - R(1,1) + 1.0 );
  q.w = 0.5 * sqrt( R(0,0) + R(1,1) + R(2,2) + 1.0 );
  return q;
}

//
// RotationMatrix std::ostream << operator
//
std::ostream& operator<<( std::ostream& out, const RotationMatrix& other ){
  out << "RotationMatrix{x:" << other.x_axis() << ",y:" << other.y_axis() << ",z:" << other.z_axis() << "}";
  return out;
}
