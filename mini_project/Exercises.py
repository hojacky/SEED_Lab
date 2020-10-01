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

def cv_exercise1(image):
    half = 50        #This will be used to resize the image to half the size
    width = int(image.shape[1] * half/100)      #Provides half the width of original image
    height = int(image.shape[0] * half/100)     #Provides half the height of original

    new_size = (width, height)     #desired size of the resized image

    res = cv2.resize(image, new_size)      #resize the image to the desired size
    
    cv2.imshow("Original", image)       #Show the original image to compare
    cv2.imshow("Resize", res)           #Show the resized image
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return res

#Mouse Callback function
def callback2(event, x, y, flags, param):
    image2 = cv2.imread('/home/pi/SEED Lab/Assignment 2/Colors_pptx.jpg')
    
    #Check for user mouse left click
    if event == cv2.EVENT_LBUTTONDOWN:
        
        #Open and load the test image
        photo = Image.open('Exercise2_test.jpg')
        pixel = photo.load()
        pixel_hsv = cv2.cvtColor(image2, cv2.COLOR_BGR2HSV)

        #print out the RGB values where the user clicks
        print("The RGB value at this point is", pixel[x,y])
        print("The HSV value at this point is", pixel_hsv[y,x])

def cv_exercise2(image2):
    cv2.namedWindow('Exercise2')
    cv2.setMouseCallback('Exercise2', callback2)

    while True:
        cv2.imshow('Exercise2', image2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        break

def cv_exercise3(image3):
    while True:

        #Convert to grayscale
        gscl = cv2.cvtColor(image3, cv2.COLOR_BGR2GRAY)

        #Showing the new image
        cv2.imshow("Original", image3)
        cv2.imshow("Grayscale", gscl)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        break

def cv_exercise4(image4):
    half = 50        #This will be used to resize the image to half the size
    width = int(image4.shape[1] * half/100)      #Provides half the width of original image
    height = int(image4.shape[0] * half/100)     #Provides half the height of original

    new_size = (width, height)     #desired size of the resized image

    image4 = cv2.resize(image4, new_size)      #resize the image to the desired size

    #Loop for converting to hsv
    while(1):
        
        #Converting to HSV
        hsv = cv2.cvtColor(image4, cv2.COLOR_BGR2HSV)
        
        #Set the range of yellow color in HSV
        lower_yellow = np.array([5,80,80])
        upper_yellow = np.array([23,255,255])

        #Grab only red colors from image
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        #Mask original image
        res = cv2.bitwise_and(image4, image4, mask=mask)
            
        while True:
            cv2.imshow('Original', image4)
            cv2.imshow('HSV', res)
            cv2.waitKey(0)
            break
        break
    cv2.destroyAllWindows()

def cv_exercise5(image, image2):
    while True:
        #convert both images to grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        cv2.imshow('Aruco', image)
        cv2.imshow("No Markers", image2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        break
    
    #get aruco dictionary and parameters to be able to detect them from photos
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    #gathers information about aruco marker scanned
    corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    #check if the id of the aruco marker exists and display id if they exist
    if ids1 == None:
        print("No markers found.")
    else:
        print("The IDs for the Aruco Markers are:")
        print(ids1)

    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(image2, aruco_dict, parameters=parameters)
    if ids2 == None:
        print("No markers found.")
    else:
        print("The IDs for the Aruco Markers are:")
        print(ids2)

def cv_exercise6():
    sleep(3)
    print("Press CTRL + C to stop scanning for Aruco Markers.")
    while True:
        try:
            with picamera.PiCamera() as camera:
                with picamera.array.PiRGBArray(camera) as output:
                    #set camera resolution so preview image is same as actual photo
                    camera.resolution = (1920, 1088)
                    #allow user to know what image is going to be taken
                    camera.start_preview()
                    sleep(4)
                    camera.stop_preview()
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
                    else:
                        print("The IDs for the Aruco Markers are:")
                        print(ids[0], ids[1])
                        #shows the image with ids of markers and corners
                        plt.figure()
                        plt.imshow(frame_markers)
                        for i in range(len(ids)):
                            c = corners[i][0]
                            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
                        plt.legend()
                        plt.show()
                    sleep(1)
                    output.truncate(0)

        #This will check if the user presses CTRL + C to end while loop
        except KeyboardInterrupt:
            print("Stopped scanning for Aruco Markers")
            break

def cv_exercise7():
    sleep(3)
    print("Press CTRL + C to stop scanning for Aruco Markers.")
    f = 1550        #focal length of camera in px
    X_real = 189    #actual length of aruco marker in px
    fov = 54        #field of view of camera
    image_length = 1920     #length of image in px
    while True:
        try:
            with picamera.PiCamera() as camera:
                with picamera.array.PiRGBArray(camera) as output:
                    #set camera resolution so preview image is same as actual photo
                    camera.resolution = (1920, 1088)
                    #allow user to know what image is going to be taken
                    camera.start_preview()
                    sleep(4)
                    camera.stop_preview()
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
                    else:
                        print("The IDs for the Aruco Markers are:")
                        print(ids[0], ids[1])
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
                            #gather position of corners and gets height
                            y1 = c[2][1] - c[0][1]
                            y2 = c[1][1] - c[3][1]
                            #gathers average of both heights
                            y = (y1 + y2)/2
                            #solve for distance using perspective projection equation
                            Z = f*X_real/y
                            #convert pixels to mm
                            Z = Z / 3.77953
                            print("The distance of id %d is %d mm" %(ids[i], Z))

                            #Angle calculation
                            x_center = (c[0][0] + c[1][0] + c[2][0] + c[3][0])/4
                            deg = (x_center / image_length)*(fov)
                            print("The angle of id %d is %d degrees." %(ids[i], deg))

                            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
                        
                        plt.legend()
                        plt.show()
                        image = cv_exercise1(image)
                    sleep(1)
                    output.truncate(0)

        #This will check if the user presses CTRL + C to end while loop
        except KeyboardInterrupt:
            print("Stopped scanning for Aruco Markers")
            break


    
#Main will run all of the exercises in order
if __name__ == '__main__':  
    print("Running Exercise 1...")
    image = cv2.imread('/home/pi/SEED Lab/Assignment 2/Exercise1_test.jpg')
    cv_exercise1(image)

    print("Running Exercise 2...")
    image = cv2.imread('/home/pi/SEED Lab/Assignment 2/Exercise2_test.jpg')
    cv_exercise2(image)

    print("Running Exercise 3...")
    image = cv2.imread('/home/pi/SEED Lab/Assignment 2/Exercise3_test.jpg')
    cv_exercise3(image)

    print("Running Exercise 4...")
    image = cv2.imread('/home/pi/SEED Lab/Assignment 2/Colors_pptx.jpg')
    cv_exercise4(image)

    print("Running Exercise 5...")
    image = cv2.imread('/home/pi/SEED Lab/Assignment 2/Exercise5_aruco.jpg')
    image2 = cv2.imread('/home/pi/SEED Lab/Assignment 2/Exercise5_nomark.jpg')
    cv_exercise5(image, image2)

    print("Running Exercise 6...")
    cv_exercise6()

    print("Running Exercise 7...")
    cv_exercise7()

    

    
