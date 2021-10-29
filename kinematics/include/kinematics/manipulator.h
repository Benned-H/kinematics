/**
  A class implementing a robot manipulator composed of rigid-body links forming a kinematic chain
  Author: Benned Hedegaard
*/

#pragma once

#include "core/vector-3d.h"
#include "kinematics/link.h"

class Manipulator{
  public:
    
    /**
      A struct representing a column of a geometric Jacobian
      Contains two Vector3Ds for contributions to both the linear and angular velocity
    */
    struct J_column_t{
      Vector3D Pi;
      Vector3D Oi;
    };

    /**
      Do not allow an empty constructor for the Manipulator class
    */
    Manipulator( void ) = delete;

    /**
      Default parameter constructor for the Manipulator class

      @brief Default parameter constructor for the Manipulator class
      @param[in]      baseFrameArg      base frame (Frame 0) for the manipulator
      @param[in]      linksArg          links of the manipulator
    */
    Manipulator( const HomogeneousTransformation& baseFrameArg, const std::vector<Link>& linksArg );

    /**
      Update the stored homogeneous transformations in each Link of the Manipulator using the current joint variables

      @brief Update the stored homogeneous transformations in each Link of the Manipulator
      @returns      none
      @throws       no expected throws
    */
    void update( void );

    /**
      Return the end-effector pose for the Manipulator

      @brief Return the end-effector pose for the Manipulator
      @returns      HomogeneousTransformation of the end-effector
      @throws       no expected throws
    */
    HomogeneousTransformation end_pose( void )const;

    /**
      Return the end-effector position (in Frame 0) for the Manipulator

      @brief Return the end-effector position (in Frame 0) for the Manipulator
      @returns      end-effector position as a Vector3D
      @throws       no expected throws
    */
    Vector3D pe( void )const;

    /**
      Return the geometric Jacobian column for the given Link (1 to n)

      @brief Return the geometric Jacobian column for the given Link (1 to n)
      @param[in]    link    number of the link whose column will be computed
      @returns      Jacobian column vector
      @throws       throws std::runtime_error on invalid column request
    */
    J_column_t get_J_column( const int i )const;

    /**
      Return the 6 x n geometric Jacobian for the entire manipulator

      @brief Return the 6 x n geometric Jacobian for the entire manipulator
      @returns      geometric Jacobian as a MatrixXd
      @throws       no expected throws
    */
    Eigen::MatrixXd J( void )const;

    /**
      Perturb the joint variables of the Manipulator by the given q_dot values

      @brief Perturb the joint variables of the Manipulator by the given q_dot values
      @param[in]    q_dot     derivative of the joint state to simulate motion using
      @returns      none
      @throws       no expected throws
    */
    void perturb( const Eigen::MatrixXd& q_dot );

    /** Contains the n links making up the manipulator, each including their own DH parameters and Joint information */
    std::vector< Link > links;

    /** Contains the n + 1 poses for each frame of the manipulator, starting with Frame 0 up to Frame n */
    std::vector< HomogeneousTransformation > frames;
  
  protected:
  private:

};
