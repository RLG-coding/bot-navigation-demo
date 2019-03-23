.Turtlebot Navigation Demo.

The objective of this project was to program a small three-wheel robot's processor in order to have it automatically navigate a virtual space. 

After manually guiding the robot through the map and saving the resulting analysis, the script, developed in Python, uses Robot Operating System (ROS) modules to allow the robot to move to any point on the saved map. The robot calculates a direct trajectory to the point which avoids any walls along the way. Additionally, the robot has been programmed to receive signals from two external sensors, an infrared motion sensor and an accelerometer.

The infrared motion sensor is connected to a Zigduino board (an Arduino-like board) on a different PC, which emits signals through the 802.15.4 protocol to a second Zigduino board connected to the robot in the virtual space when motion is detected.

The accelerometer is connected to a LoPy board, which emits signals through the LoRa protocol to the robot's processor. Some of the files on the LoPy board have been modified to ensure signals are only sent when the board is moved.

When a signal is received from either sensor, the robot completes a specific, pre-determined task - moving to a given point on the map, in this case.