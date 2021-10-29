/**
  A class implementing a 4x4 transformation matrix representing a 3D transformation
  The top left 3x3 elements of the matrix form a rotation matrix and the top right vertical 3-vector forms a position w.r.t. the previous frame

  Author: Benned Hedegaard
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "geometry_msgs/Transform.h"
#include "core/vector-3d.h"
#include "core/rotation-matrix.h"

class HomogeneousTransformation{
  public:
    /**
      Default empty constructor for the HomogeneousTransformation class
    */
    HomogeneousTransformation( void ) = default;

    /**
      Identity transformation matrix constructor

      @brief Identity transformation matrix constructor
      @returns      identity homogeneous transformation matrix
      @throws       no expected throws
    */
    static HomogeneousTransformation identity( void );

    /**
      DH parameter constructor for the HomogeneousTransformation class
      
      These parameters represent the transformation from Frame i-1 to Frame i through Joint i
      See Siciliano Ch. 2.8.2, specifically Eq. 2.52

      @brief DH parameter constructor for the HomogeneousTransformation class
      @param[in]    a       distance from frame Oi' to Oi
      @param[in]    alpha   angle from z(i-1) axis to zi axis about xi axis
      @param[in]    d       distance from frame O(i-1) to Oi' along z(i-1) axis
      @param[in]    theta   angle from x(i-1) axis to xi axis about z(i-1) axis
      @returns      corresponding homogeneous transformation
      @throws       no expected throws
    */
    static HomogeneousTransformation fromDH( const double a, const double alpha, const double d, const double theta ); 

    /**
      Parameter constructor for the HomogeneousTransformation class

      @brief Parameter constructor for the HomogeneousTransformation class
      @param[in]    v     position of the transformation
      @param[in]    rm    orientation of the transformation
      @returns      none
      @throws       no expected throws
    */
    HomogeneousTransformation( const Vector3D& v, const RotationMatrix& rm );

    /**
      Parameter constructor from Eigen::Matrix4d for the HomogeneousTransformation class

      @brief Parameter constructor from Eigen::Matrix4d for the HomogeneousTransformation class
      @param[in]    tArg    Eigen::Matrix4d used to populate the transformation
      @returns      none
      @throws       no expected throws
    */
    HomogeneousTransformation( const Eigen::Matrix4d& tArg );
 
    /**
      Default destructor for the HomogeneousTransformation class
    */
    virtual ~HomogeneousTransformation( void ) = default;

    /**
      HomogeneousTransformation set function

      @brief HomogeneousTransformation set function
      @param[in]    v     assigned to the position of the transformation
      @param[in]    rm    assigned to the orientation of the transformation
      @returns      none
      @throws       no expected throws
    */
    void set( const Vector3D& v, const RotationMatrix& rm );

    /**
      Return the position of the HomogeneousTransformation as a Vector3D

      @brief Return the position of the HomogeneousTransformation as a Vector3D
      @returns    position of the transformation
      @throws     no expected throws
    */
    Vector3D position( void )const;

    /**
      Return the orientation of the HomogeneousTransformation as a RotationMatrix

      @brief Return the orientation of the HomogeneousTransformation as a RotationMatrix
      @returns    orientation of the transformation
      @throws     no expected throws
    */
    RotationMatrix orientation( void )const;

    /**
      Return the z-axis of the HomogeneousTransformation as a Vector3D

      @brief Return the z-axis of the HomogeneousTransformation as a Vector3D
      @returns    z-axis of the rotation in the transformation
      @throws     no expected throws
    */
    Vector3D z_axis( void )const;

    /**
      Return the inverse of the HomogeneousTransformation

      @brief Return the inverse of the HomogeneousTransformation
      @returns    inverse of the transformation
      @throws     no expected throws
    */
    HomogeneousTransformation inverse( void )const;

    /**
      Return the HomogeneousTransformation as a geometry_msgs::Transform

      @brief Return the HomogeneousTransformation as a geometry_msgs::Transform
      @returns    geometry_msgs::Transform version of the transformation
      @throws     no expected throws
    */
    geometry_msgs::Transform to_Transform( void )const;

    /**
      HomogeneousTransformation multiplication

      @brief HomogeneousTransformation multiplication
      @param[in]    lhs     left matrix in the product
      @param[in]    rhs     right matrix in the product
      @returns      matrix product of the two transformations
      @throws       no expected throws
    */
    friend inline HomogeneousTransformation operator*( const HomogeneousTransformation& lhs, const HomogeneousTransformation& rhs ){
      return HomogeneousTransformation( lhs.T * rhs.T );
    }

    /**
      HomogeneousTransformation std::ostream << operator

      @brief HomogeneousTransformation std::ostream << operator
      @param[in]    out     output stream printed to
      @param[in]    other   transformation to be printed
      @returns      updated output stream with the transformation printed
      @throws       no expected throws
    */
    friend std::ostream& operator<<( std::ostream& out, const HomogeneousTransformation& other ){
      out << "HomogeneousTransformation{" << other.T << "}";
      return out;
    }

  protected:
    /** 4x4 homogeneous matrix representing a position and orientation w.r.t. some reference frame */
    Eigen::Matrix4d T;
  
  private:

};
