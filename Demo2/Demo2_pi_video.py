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

def marker_detection():
    ret, image = cap.read()
              
    #Convert image to grayscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    
    #gathers aruco dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                    
    #checks if any aruco markers were detected
    if ids == None:
        print("No markers found.")
        return False
    else:
        return True

def angle_detection():
    fov = 54        #field of view of camera
    image_length = 1920     #length of image in px
    
    ret, image = cap.read()
              
    #Convert image to grayscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    
    #gathers aruco dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                    
    #checks if any aruco markers were detected
    if ids == None:
        print("No markers found.")
        return False, 0
    else:       
        for i in range(len(ids)):
            c = corners[i][0]
        
            #Angle calculation
            x_center = (c[0][0] + c[1][0] + c[2][0] + c[3][0])/4
            deg = (x_center / image_length)*(fov)
                            
            #check for position of marker on the wheel
            if deg < 27:
                deg = -27 + deg
                print(deg)
                return True, deg
            elif deg > 27:
                deg = deg - 27
                print(deg)
                return True, deg
            
def dist_detection():
    f = 1550        #focal length of camera in px
    X_real = 697.32    #actual length of aruco marker in px

    ret, image = cap.read()
              
    #Convert image to grayscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                    
    #gathers aruco dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                    
    #checks if any aruco markers were detected
    if ids == None:
        print("No markers found.")
        return False
    else:
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
            #convert pixels to mm then feet
            Z = Z / 3.77953
            Z = Z/305
            print("The distance of id %d is %d ft" %(ids[i], Z))
            return True, Z

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
    #setup for video and resolution
    cap = cv2.VideoCapture(0)
    cap.set(3, 1920)
    cap.set(4,1080)
	
    marker_found = False
    angle_found = False
    dist_found = False
    
    #continue video capture until a marker is found
    print("Scanning for markers")
    while (marker_found == False):
        marker_found = marker_detection()
    writeNumber(0)      #write to arduino to tell it to stop motors
    while (readNumber != 1):        #wait until the motor stops and arduino sends a number to tell the pi to start looking for angle
        pass
    print("Start detecting for angle")
    
    #detect the angle of the aruco marker
    while (angle_found == False):
        angle_found, angle = angle_detection()
    
    writeNumber(2)
    sleep(2)
    writeNumber(angle)      #write angle to arduino
    #wait until the robot is centered onto the marker
    while (readNumber != 1):
        pass
    print("Start detecting for distance")
    
    #detect the distance of the aruco marker
    while (dist_found == False):
        dist_found, distance = dist_detection()
    writeNumber(3)
    sleep(2)
    writeNumber(distance)
    
        
    


    
            

