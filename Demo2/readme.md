Demo 2:

The team coded the robot to take turn in a circle and detect an aruco marker. The Pi then calculates and angle and distance to send via the I2C bus to the Arduino. 
The Arduino program tells the robot to turn the number of degrees, then travel a foot in front of the beacon, and then drive in a circle around the beacon.
