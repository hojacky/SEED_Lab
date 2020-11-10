# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera, Color
from time import sleep
from PIL import Image
import cv2
import cv2.aruco as aruco
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def angle_detection():
    fov = 54        #field of view of camera
    image_length = 1920     #length of image in px
	f = 1550        #focal length of camera in px
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
    X_real = 189    #actual length of aruco marker in px CHANGE TO THE ACTUAL LENGTH OF THE MARKER FROM THE DEMO
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
	##########################################
    
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as output:
            #set camera resolution so preview image is same as actual photo
            camera.resolution = (1920, 1088)
                    
            #allow user to know what image is going to be taken
            camera.start_preview()
            sleep(3)
            camera.stop_preview()
                    
            #g is the average awb_gains red and blue values
            g = (1.556, 1.168)
            camera.awb_mode = 'off'
            camera.awb_gains = g
            camera.capture(output, 'rgb')
                    
            #set the image taken to image variable and convert to grayscale
            image = output.array
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    
            #gathers aruco dictionary and parameters
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()

            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                    
            #checks if any aruco markers were detected
            if ids == None:
                print("No markers found.")
				return 100000000, 100000000000
            else:
                print("The IDs for the Aruco Markers are:")
                for i in range(len(ids)):
                    print(ids[i])
					
                for i in range(len(ids)):
                    c = corners[i][0]
					
					#Distance calculation
					#gather position of corners and gets height
					y1 = c[2][1] - c[0][1]
					y2 = c[1][1] - c[3][1]
					#gathers average of both heights
					y = (y1 + y2)/2
					if y < 0:
					y = y * -1
					#solve for distance using perspective projection equation
							#DON'T FORGET TO CHANGE X_real
					Z = f*X_real/y
					#convert pixels to mm
					Z = Z / 3.77953
					print("The distance of id %d is %d mm" %(ids[i], Z))

                    #Angle calculation
                    x_center = (c[0][0] + c[1][0] + c[2][0] + c[3][0])/4
                    deg = (x_center / image_length)*(fov)
                            
                    #check for position of marker on the wheel
                    if deg < 27:
                        deg = -27 + deg
                        print(deg)
                        return deg, Z
                    elif deg > 27:
                        deg = deg - 27
                        print(deg)
                        return deg, Z
                            
            output.truncate(0)

#Set up for sending values to Arduino
bus = smbus.SMBus(1)

#This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    bus.write_byte(address,value)
    #bus.write_byte_data(address,0,value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number
        
if __name__ == '__main__':

    while True:
            
    angle, distance = aruco_detection()
	
	if (deg < 100000000) and (distance < 100000000000) and (distance > 0):
	
		writeNumber(angle)
		sleep(1)
	
		writeNumber(distance)
		sleep(1)
		
		print("Press any key to take another image.")
		cv2.waitKey(0)
	
	else:
		continue
	
            
