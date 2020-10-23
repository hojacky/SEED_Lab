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
                sleep(5)
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

                        #Angle calculation
                        x_center = (c[0][0] + c[1][0] + c[2][0] + c[3][0])/4
                        deg = (x_center / image_length)*(fov)

                        y_avg = (c[0][1]+c[1][1]+c[2][1]+c[3][1])/4
                            
                        #check for position of marker on the wheel
                        if deg < 27:
                            deg = -27 + deg
                            print(deg)
                            return deg
                        elif deg > 27:
                            deg = deg - 27
                            print(deg)
                            return deg
                            
                output.truncate(0)
                
    except:
        print("Done")
        
if __name__ == '__main__':

    while True:
        
        try:

            angle = aruco_detection()
            sleep(2)
            
        except KeyboardInterrupt:
            print("Stopped scanning for Aruco Markers")
            break
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
    scroll_msg1 = ("Angle Detected: "+ str(angle))
    lcd.message = scroll_msg1
    for i in range(len(scroll_msg1)):
        time.sleep(0.5)
        lcd.move_left()
    # Wait 5s
    time.sleep(1)
    # Set LCD color to purple
    lcd.color = [50, 0, 50]
    time.sleep(1)
    #clear before print new message
    lcd.clear()

    # Turn off LCD backlights and clear text
    lcd.color = [0, 0, 0]        


