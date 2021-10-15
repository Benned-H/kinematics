/**
  A class implementing a 4x4 transformation matrix representing a 3D transformation
  The top left 3x3 elements of the matrix form a rotation matrix and the top right vertical 3-vector forms a position w.r.t. the previous frame

  Author: Benned Hedegaard
*/

#include "core/homogeneous-transformation.h"

//
// Identity transformation matrix constructor
//
HomogeneousTransformation HomogeneousTransformation::identity( void ){
  return HomogeneousTransformation( Eigen::Matrix< double, 4, 4 >::Identity() );
}

//
// DH parameter constructor for the HomogeneousTransformation class
//   
HomogeneousTransformation HomogeneousTransformation::fromDH( const double a, const double alpha, const double d, const double theta ){
  double stheta = sin( theta );
  double ctheta = cos( theta );
  double salpha = sin( alpha );
  double calpha = cos( alpha );

  // Based on Eq. 2.52 of Siciliano
  Eigen::Matrix4d T;
  T << ctheta, -stheta*calpha, stheta*salpha, a*ctheta,
       stheta, ctheta*calpha, -ctheta*salpha, a*stheta,
       0.0, salpha, calpha, d,
       0.0, 0.0, 0.0, 1.0;

  return HomogeneousTransformation( T );
}

//
// Parameter constructor for the HomogeneousTransformation class
//
HomogeneousTransformation::HomogeneousTransformation( const Vector3D& v, const RotationMatrix& rm ) : T(4,4){
  set( v, rm );
}
//
// Parameter constructor from Eigen::Matrix4d for the HomogeneousTransformation class
//
HomogeneousTransformation::HomogeneousTransformation( const Eigen::Matrix4d& tArg ) : T( tArg ){

}

//
// HomogeneousTransformation set function
//
void HomogeneousTransformation::set( const Vector3D& v, const RotationMatrix& rm ){
  T(0,0) = rm(0,0);
  T(0,1) = rm(0,1);
  T(0,2) = rm(0,2);
  T(0,3) = v.x();
  T(1,0) = rm(1,0);
  T(1,1) = rm(1,1);
  T(1,2) = rm(1,2);
  T(1,3) = v.y();
  T(2,0) = rm(2,0);
  T(2,1) = rm(2,1);
  T(2,2) = rm(2,2);
  T(2,3) = v.z();
  T(3,0) = 0.0;
  T(3,1) = 0.0;
  T(3,2) = 0.0;
  T(3,3) = 1.0;
}

//
// Return the position of the HomogeneousTransformation as a Vector3D
//
Vector3D HomogeneousTransformation::position( void )const{
  return Vector3D( T(0,3), T(1,3), T(2,3) );
}

//
// Return the orientation of the HomogeneousTransformation as a RotationMatrix
//
RotationMatrix HomogeneousTransformation::orientation( void )const{
  Eigen::Matrix3d R;
  R << T(0,0), T(0,1), T(0,2),
       T(1,0), T(1,1), T(1,2),
       T(2,0), T(2,1), T(2,2);
  return RotationMatrix( R );
}

//
// Return the inverse of the HomogeneousTransformation
//
HomogeneousTransformation HomogeneousTransformation::inverse( void )const{
  RotationMatrix inverse_R = orientation().inverse();
  Vector3D inverse_p = -( inverse_R * position() );
  return HomogeneousTransformation( inverse_p, inverse_R );
}

//
// Return the HomogeneousTransformation as a geometry_msgs::Transform
//
geometry_msgs::Transform HomogeneousTransformation::to_Transform( void )const{
  geometry_msgs::Transform transform;
  transform.translation.x = T(0,3);
  transform.translation.y = T(1,3);
  transform.translation.z = T(2,3);
  transform.rotation = orientation().to_Quaternion();
  return transform;
}
