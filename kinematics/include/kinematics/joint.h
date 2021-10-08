/**
  Class to represent a joint, either revolute or prismatic, in a kinematic chain
  Author: Benned Hedegaard
*/

#pragma once

class Joint{

  public:
    enum JointType { Revolute, Prismatic };

    /**
      Do not allow empty constructor for the Joint class
    */
    Joint( void ) = delete;

    /**
      Default parameter constructor for the Joint class
    
      @brief Default parameter constructor for the Joint class
      @param[in]    valueArg      initial value for the joint variable
      @param[in]    typeArg       type of the joint (revolute or prismatic)
      @returns      none
      @throws       no expected throws
    */
    Joint( const double valueArg, const JointType typeArg );

    /** Stores the current joint variable value of the Joint */
    double value;

    /** Indicates whether this Joint is revolute or prismatic */
    JointType type;

  protected:
  private:
};
