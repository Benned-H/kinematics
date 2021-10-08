/**
  A class implementing a 3D vector of the form (x,y,z)
  Author: Benned Hedegaard
*/

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

class Vector3D{
  public:
    /**
      Empty constructor for the Vector3D class

      @brief Empty constructor for the Vector3D class
      @returns    none
      @throws     no expected throws
    */
    Vector3D( void );

    /**
      Parameter constructor from (x,y,z) for the Vector3D class

      @brief Parameter constructor from (x,y,z) for the Vector3D class
      @param[in]    x     value for x in the vector
      @param[in]    y     value for y in the vector
      @param[in]    z     value for z in the vector
      @returns      none
      @throws       no expected throws
    */
    Vector3D( const double& x, const double& y, const double& z );
    
    /**
      Parameter constructor from Eigen::Vector3d for the Vector3D class

      @brief Parameter constructor from Eigen::Vector3d for the Vector3D class
      @param[in]    vArg    Eigen::Vector3d used to construct the Vector3D
      @returns      none
      @throws       no expected throws
    */
    Vector3D( const Eigen::Vector3d& vArg );

    /**
      Vector3D cast to Eigen::Vector3d

      @brief Vector3D cast to Eigen::Vector3d
      @returns      Eigen::Vector3d version of the vector
      @throws       no expected throws
    */
    Eigen::Vector3d to_Vector3d( void )const;

    /**
      Default destructor for the Vector3D class
    */
    virtual ~Vector3D( void ) = default;

    /**
      Vector3D set function

      @brief Vector3D set function
      @param[in]    x     value assigned to the x element of the vector
      @param[in]    y     value assigned to the y element of the vector
      @param[in]    z     value assigned to the z element of the vector
      @returns      none
      @throws       no expected throws
    */
    void set( const double& x, const double& y, const double& z );

    /**
      Vector3D in-place element-wise addition

      @brief Vector3D in-place element-wise addition
      @param[in]    rhs   Vector3D whose elements are added to this Vector3D
      @returns      none
      @throws       no expected throws
    */
    void operator+=( const Vector3D& rhs );
    
    /**
      Vector3D in-place element-wise subtraction

      @brief Vector3D in-place element-wise subtraction
      @param[in]    rhs   Vector3D whose elements are subtracted from this Vector3D
      @returns      none
      @throws       no expected throws
    */
    void operator-=( const Vector3D& rhs );

    /**
      Return the L2 norm of the Vector3D

      @brief Return the L2 norm of the Vector3D
      @returns      L2 norm (length) of the Vector3D
      @throws       no expected throws
    */
    double norm( void )const;

    /**
      Return the x element of the Vector3D

      @brief Return the x element of the Vector3D
      @returns      x value of the vector
      @throws       no expected throws
    */
    double x( void )const;

    /**
      Return the y element of the Vector3D

      @brief Return the y element of the Vector3D
      @returns      y value of the vector
      @throws       no expected throws
    */
    double y( void )const;

    /**
      Return the z element of the Vector3D

      @brief Return the z element of the Vector3D
      @returns      z value of the vector
      @throws       no expected throws
    */
    double z( void )const;

    /**
      Vector3D unary operator- which represents negation

      @brief Vector3D unary operator- which represents negation
      @param[in]    rhs   Vector3D to be negated
      @returns      negated Vector3D
      @throws       no expected throws
    */
    friend inline Vector3D operator-( const Vector3D& rhs ){
      return Vector3D( -rhs.v[0], -rhs.v[1], -rhs.v[2] );
    }

    /**
      Vector3D element-wise addition

      @brief Vector3D element-wise addition
      @param[in]    lhs   left Vector3D in the sum
      @param[in]    rhs   right Vector3D in the sum
      @returns      Vector3D element-wise sum of the inputs
      @throws       no expected throws
    */
    friend inline Vector3D operator+( const Vector3D& lhs, const Vector3D& rhs ){
      return Vector3D( lhs.v[0] + rhs.v[0], lhs.v[1] + rhs.v[1], lhs.v[2] + rhs.v[2] );
    }

    /**
      Vector3D element-wise subtraction

      @brief Vector3D element-wise subtraction
      @param[in]    lhs   left Vector3D in the difference
      @param[in]    rhs   right Vector3D in the difference
      @returns      Vector3D element-wise difference of the inputs
      @throws       no expected throws
    */
    friend inline Vector3D operator-( const Vector3D& lhs, const Vector3D& rhs ){
      return Vector3D( lhs.v[0] - rhs.v[0], lhs.v[1] - rhs.v[1], lhs.v[2] - rhs.v[2] );
    }
  
  protected:
    /** Vector of the form (x,y,z) representing a 3D position */
    std::vector<double> v;
  
  private:

}; 

/**
  Vector3D std::ostream << operator

  @brief Vector3D std::ostream << operator
  @param[in]    out     std::ostream printed to
  @param[in]    other   Vector3D to be printed
  @returns      updated output stream with the Vector3D printed
  @throws       no expected throws
*/
std::ostream& operator<<( std::ostream& out, const Vector3D& other );
