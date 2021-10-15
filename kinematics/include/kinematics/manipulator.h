/**
  A class implementing a robot manipulator composed of rigid-body links forming a kinematic chain
  Author: Benned Hedegaard
*/

#pragma once

#include "kinematics/link.h"

class Manipulator{
  public:

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
      Updates the stored homogeneous transformations in each Link of the Manipulator using the current joint variables

      @brief Updates the stored homogeneous transformations in each Link of the Manipulator
      @returns      none
      @throws       no expected throws
    */
    void update( void );

    /**
      Returns the end-effector pose for the Manipulator

      @brief Returns the end-effector pose for the Manipulator
      @returns      HomogeneousTransformation of the end-effector
      @throws       no expected throws
    */
    HomogeneousTransformation end_pose( void )const;

    /** Contains the n links making up the manipulator, each including their own DH parameters and Joint information */
    std::vector< Link > links;

    /** Contains the n + 1 poses for each frame of the manipulator, starting with Frame 0 up to Frame n */
    std::vector< HomogeneousTransformation > frames;
  
  protected:
  private:

};
