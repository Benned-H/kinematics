/**
  A class implementing a 3D vector of the form (x,y,z)
  Author: Benned Hedegaard
*/

#include <math.h>
#include "core/vector-3d.h"

//
// Empty constructor for the Vector3D class
//
Vector3D::Vector3D( void ) : v( 3, 0.0 ){}

//
// Parameter constructor from (x,y,z) for the Vector3D class
//
Vector3D::Vector3D( const double x, const double y, const double z ) : v(){
  v.push_back( x );
  v.push_back( y );
  v.push_back( z );
}

//
// Parameter constructor from Eigen::Vector3d for the Vector3D class
//
Vector3D::Vector3D( const Eigen::Vector3d& vArg ) : v(){
  v.push_back( vArg(0) );
  v.push_back( vArg(1) );
  v.push_back( vArg(2) );
}

//
// Vector3D cast to Eigen::Vector3d
//
Eigen::Vector3d Vector3D::to_Vector3d( void )const{
  return Eigen::Vector3d( v[0], v[1], v[2] );
}

//
// Vector3D set function
//
void Vector3D::set( const double x, const double y, const double z ){
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

//
// Vector3D in-place element-wise addition
//
void Vector3D::operator+=( const Vector3D& rhs ){
  v[0] += rhs.v[0];
  v[1] += rhs.v[1];
  v[2] += rhs.v[2];
}

//
// Vector3D in-place element-wise subtraction
//
void Vector3D::operator-=( const Vector3D& rhs ){
  v[0] -= rhs.v[0];
  v[1] -= rhs.v[1];
  v[2] -= rhs.v[2];
}

//
// Return the L2 norm of the Vector3D
//
double Vector3D::norm( void )const{
  return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}

//
// Return the x element of the Vector3D
//
double Vector3D::x( void )const{
  return v[0];
}

//
// Return the y element of the Vector3D
//
double Vector3D::y( void )const{
  return v[1];
}

//
// Return the z element of the Vector3D
//
double Vector3D::z( void )const{
  return v[2];
}

//
// Vector3D std::ostream << operator
//
std::ostream& operator<<( std::ostream& out, const Vector3D& other ){
  out << "[" << other.x() << "," << other.y() << "," << other.z() << "]";
  return out;
}
