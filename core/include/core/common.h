/**
 * Common functions for the core package
 * Author: Benned Hedegaard
 */

/**
 * Returns the sign of the given double (see Siciliano pg. 55)
 */
inline double sign( const double x ){
  if( x < 0.0 ){
    return -1.0;
  }
  return 1.0;
}
