/**
  A class implementing a link in a robot manipulator with a single included joint
  Author: Benned Hedegaard
*/

#pragma once

#include "kinematics/joint.h"
#include "core/homogeneous-transformation.h"

class Link{
  public:

    /**
      A struct representing the DH parameters for a link
    
      a - Distance from Oi' to Oi
      alpha - Angle from z(i-1) to zi about xi.
      d - Coordinate of Oi' along z(i-1)
      theta - Angle from x(i-1) to xi about z(i-1)
    */
    struct dh_parameters_t{
      double a;
      double alpha;
      double d;
      double theta;
    };

    /**
      Default parameter constructor for the Link class

      @brief Default parameter constructor for the Link class
      @param[in]    dhArg           DH parameters defining the joint kinematics
      @param[in]    jointTypeArg    type of the link's joint (revolute or prismatic)
      @returns      none
      @throws       std::runtime_error on unexpected JointType
    */
    Link( const dh_parameters_t& dhArg, const Joint::JointType& jointTypeArg );

    /**
      Updates the link transformation based on the preceding Joint's state

      @brief Updates the link transformation based on the preceding Joint's state
      @returns      none
      @throws       std::runtime_error on unexpected JointType
    */
    void update( void );

    /** The current transformation from Frame i-1 to Frame i based on Joint i */
    HomogeneousTransformation transformation;

  protected:
    
    /** The Joint preceding this Link in the kinematic chain, both numbered i */
    Joint joint;

    /** struct containing the four DH parameters for this Link */
    dh_parameters_t params;
  
 private:

};
