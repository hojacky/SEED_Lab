# Author: Caroline Hidalgo
# October 3 2020
# Miniproject: Determining Desired Motor Position + Display LCD from Arduino
import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


 ################# FUNCTION ARUCO_DETECTION ##################
# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera, Color
from time import sleep
from PIL import Image
import time
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

def aruco_detection():
    print("Press CTRL + C to stop scanning for Aruco Markers.")
    f = 1550        #focal length of camera in px
    X_real = 189    #actual length of aruco marker in px
    fov = 54        #field of view of camera
    image_length = 1920     #length of image in px
    location = 0    #This will tell the arduino which location the aruco marker was at
    try:
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
                #g = (1.293, 1.586)
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
                    location = 0
                    return location
                else:
                    print("The IDs for the Aruco Markers are:")
                    for i in range(len(ids)):
                        print(ids[i])
                    #shows the image with ids of markers and corners
                    plt.figure()
                    plt.imshow(frame_markers)
                    for i in range(len(ids)):
                        c = corners[i][0]

                        #Distance calculation
                        #gathers position of corners and gets length
                        x1 = c[0][0] - c[2][0]
                        x2 = c[1][0] - c[3][0]
                        #gathers average of both lengths
                        x = (x1 + x2)/2
                        if x < 0:
                            x = x * -1
                        #gather position of corners and gets height
                        y1 = c[2][1] - c[0][1]
                        y2 = c[1][1] - c[3][1]
                        #gathers average of both heights
                        y = (y1 + y2)/2
                        if y < 0:
                            y = y * -1
                        #solve for distance using perspective projection equation
                        Z = f*X_real/y
                        #convert pixels to mm
                        Z = Z / 3.77953
                        #print("The distance of id %d is %d mm" %(ids[i], Z))

                        #Angle calculation
                        x_center = (c[0][0] + c[1][0] + c[2][0] + c[3][0])/4
                        deg = (x_center / image_length)*(fov)
                        #print("The angle of id %d is %d degrees." %(ids[i], deg))

                        x_avg = (c[0][0]+c[1][0]+c[2][0]+c[3][0])/4
                        y_avg = (c[0][1]+c[1][1]+c[2][1]+c[3][1])/4
                            
                        #check for position of marker on the wheel
                        if deg < 27 and y_avg > 544:
                            print("The marker is at 3 position.")
                            location = 3
                            return location
                        elif deg < 27 and y_avg < 544:
                            print("The marker is at 4 position.")
                            location = 4
                            return location
                        elif deg > 27 and y_avg > 544:
                            print("The marker is at 2 position.")
                            location = 2
                            return location
                        elif deg > 27 and y_avg < 544:
                            print("The marker is at 1 position.")
                            location = 1
                            return location
                        else:
                            location = 0
                            return location
                            
                output.truncate(0)
    except:
        print("error")



##############Exercise 1- Send, Add five, Recieve###############

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
        try:
            #Calculating Desired Position

            x = aruco_detection();
            
            var = x;


            writeNumber(var)
            print ("RPI: Hi Arduino, I sent you" , var)
            # sleep one second
            time.sleep(1)

            number = readNumber();
            print ("Arduino: Hey RPI, I received a digit", number)
            
        
        except KeyboardInterrupt:
            print("Stopped scanning for Aruco Markers")
            break
            




    ############# Exercise 2 - Display on LCD ###############
        # Modify this if you have a different sized Character LCD
        lcd_columns = 16
        lcd_rows = 2

        # Initialise I2C bus.
        i2c = busio.I2C(board.SCL, board.SDA)

        # Initialise the LCD class
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

        lcd.clear()
        # Set LCD color to red
        lcd.color = [100, 0, 0]
        time.sleep(1)
        
        # Print message on sent value
        scroll_msg1 = ("Desired Position : "+ str(var))
        lcd.message = scroll_msg1
        for i in range(len(scroll_msg1)):
            time.sleep(0.5)
            lcd.move_left()
        # Wait 5s
        time.sleep(5)
        # Set LCD color to purple
        lcd.color = [50, 0, 50]
        time.sleep(1)
        #clear before print new message
        lcd.clear()

        # Print What digit is recieved
        scroll_msg2 = ("Recieved Position: "+ str(number))
        lcd.message = scroll_msg2
        for i in range (len(scroll_msg2)):
            time.sleep(0.5)
            lcd.move_left()
            
        time.sleep(5)

        #clear before print new message
        lcd.clear()
        lcd.blink = True
        lcd.message = "Bye!"
        time.sleep(5)
        lcd.blink = False
        lcd.clear()
        
        time.sleep(5)
        # Turn off LCD backlights and clear text
        lcd.color = [0, 0, 0]

