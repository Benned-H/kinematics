#!/usr/bin/env python

# All Python ROS nodes will have the above line which clarifies that the file should be executed as a Python script

import serial
import rospy # Allows use of ROS node functionalities
from control.msg import JointCommand

class ArduinoInterface:

  def __init__(self):
    # Serial takes two parameters: serial device and baudrate
    ser = serial.Serial('/dev/ttyUSB0', 9600)

  # Callback function for commands received from the node's subscription
  def callback(data):
    print('I received some data:', data)

def main():
  try:
    interface = ArduinoInterface()

    rospy.init_node('arduino-interface', anonymous=True)

    # Create ROS subscriber to receive joint angle commands
    command_sub = rospy.Subscriber('control/command', JointCommand, interface.callback)

    rospy.spin() # Keep Python from exiting until the node is stopped

  except serial.serialutil.SerialException: # Catch when the serial device is not recognized
    print('Error with the serial device! Is the Arduino Mega plugged in?')

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException: # Catch these exceptions when Ctrl-C is pressed to kill the node
    pass # Do nothing
