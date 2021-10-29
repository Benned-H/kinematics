/**
 * Common functions for the core package
 * Author: Benned Hedegaard
 */

#include <Eigen/Dense>

/**
 * Returns the sign of the given double (see Siciliano pg. 55)
 */
inline double sign( const double x ){
  if( x < 0.0 ){
    return -1.0;
  }
  return 1.0;
}

/**
 * Returns the result of applying the skew-symmetric operator to the given Vector3D (see Siciliano Eq. 3.9)
 */
inline Eigen::Matrix3d S( const Vector3D& v ){
  Eigen::Matrix3d output;
  output << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
  return output;
}
