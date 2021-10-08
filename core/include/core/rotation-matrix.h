/**
  A class implementing a 3x3 rotation matrix representing orientation
  Each column of the matrix corresponds to the x, y, and z axis of the relative frame w.r.t. the previous frame

  Author: Benned Hedegaard
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "core/vector-3d.h"

class RotationMatrix{
  public:
    /**
      Do not allow an empty constructor for the RotationMatrix class
    */
    RotationMatrix( void ) = delete;

    /**
      Identity rotation matrix constructor

      @brief Identity rotation matrix constructor
      @returns      identity rotation matrix
      @throws       no expected throws
    */
    static RotationMatrix identity( void );

    /**
      Vector3D parameter constructor for the RotationMatrix class

      @brief Vector3D parameter constructor for the RotationMatrix class
      @param[in]    x_axis    x-axis of the constructed RotationMatrix
      @param[in]    y_axis    y-axis of the constructed RotationMatrix
      @param[in]    z_axis    z-axis of the constructed RotationMatrix
      @returns      none
      @throws       no expected throws
    */
    RotationMatrix( const Vector3D& x_axis, const Vector3D& y_axis, const Vector3D& z_axis );

    /**
      Matrix3d parameter constructor for the RotationMatrix class

      @brief Matrix3d parameter constructor for the RotationMatrix class
      @param[in]    rArg      3x3 Eigen::Matrix3d used to construct the RotationMatrix
      @returns      none
      @throws       no expected throws
    */
    RotationMatrix( const Eigen::Matrix3d& rArg );
    
    /**
      Default destructor for the RotationMatrix class
    */
    virtual ~RotationMatrix( void ) = default;

    /**
      RotationMatrix set function

      @brief RotationMatrix set function
      @param[in]    x_axis    Vector3D assigned to the x-axis of the RotationMatrix
      @param[in]    y_axis    Vector3D assigned to the y-axis of the RotationMatrix
      @param[in]    z_axis    Vector3D assigned to the z-axis of the RotationMatrix
      @returns      none
      @throws       no expected throws
    */
    void set( const Vector3D& x_axis, const Vector3D& y_axis, const Vector3D& z_axis );

    /**
      RotationMatrix element access function

      @brief RotationMatrix element access function
      @param[in]    r     row of the accessed element in the matrix
      @param[in]    c     column of the accessed element in the matrix
      @returns      element (r,c) of the RotationMatrix
      @throws       no expected throws
    */
    double operator()( const int r, const int c )const;

    /**
      Return the x-axis of the RotationMatrix

      @brief Return the x-axis of the RotationMatrix
      @returns      x-axis of the matrix
      @throws       no expected throws
    */
    Vector3D x_axis( void )const;

    /**
      Return the y-axis of the RotationMatrix

      @brief Return the y-axis of the RotationMatrix
      @returns      y-axis of the matrix
      @throws       no expected throws
    */
    Vector3D y_axis( void )const;

    /**
      Return the z-axis of the RotationMatrix

      @brief Return the z-axis of the RotationMatrix
      @returns      z-axis of the matrix
      @throws       no expected throws
    */
    Vector3D z_axis( void )const;

    /**
      Return the inverse of the RotationMatrix

      @brief Return the inverse of the RotationMatrix
      @returns      inverse RotationMatrix
      @throws       no expected throws
    */
    RotationMatrix inverse( void )const;

    /**
      RotationMatrix multiplication

      @brief RotationMatrix multiplication
      @param[in]    lhs   left RotationMatrix in the product
      @param[in]    rhs   right RotationMatrix in the product
      @returns      RotationMatrix product of the inputs
      @throws       no expected throws
    */
    friend inline RotationMatrix operator*( const RotationMatrix& lhs, const RotationMatrix& rhs ){
      return RotationMatrix( lhs.R * rhs.R );
    }

    /**
      RotationMatrix postmultiplication with Vector3D

      @brief RotationMatrix postmultiplication with Vector3D
      @param[in]    lhs   RotationMatrix in the product
      @param[in]    rhs   Vector3D in the product
      @returns      Vector3D product
      @throws       no expected throws
    */
    friend inline Vector3D operator*( const RotationMatrix& lhs, const Vector3D& rhs ){
      return Vector3D( lhs.R * rhs.to_Vector3d() );
    }

  protected:
    /** 3x3 matrix with columns representing the x-axis, y-axis, and z-axis of the rotated frame w.r.t. the previous frame */
    Eigen::Matrix3d R;
  
  private:

}; 

/**
  RotationMatrix std::ostream << operator

  @brief RotationMatrix std::ostream << operator
  @param[in]      out     std::ostream to which the RotationMatrix is printed
  @param[in]      other   RotationMatrix to be printed
  @returns        updated output stream with the RotationMatrix printed
  @throws         no expected throws
*/
std::ostream& operator<<( std::ostream& out, const RotationMatrix& other );
