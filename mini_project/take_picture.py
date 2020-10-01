#This code is in Python
#This code takes a picture using the camera on the Pi
# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera, Color
from time import sleep
import time
import cv2

if __name__ == '__main__':
 
   fileName = input("File Name:")
   fileName += ".jpg"

   # initialize the camera and grab a reference to the raw camera capture
   camera = PiCamera()
   rawCapture = PiRGBArray(camera)
 
   # allow the camera to warmup
   camera.resolution = (1920, 1088)
   camera.framerate = 30
   camera.start_preview()
   sleep(5)
   g = camera.awb_gains
   camera.awb_mode = 'off'
   camera.awb_gains = g
   camera.stop_preview()
   
   # grab an image from the camera
   print("Capturing Image...")
   try:
      camera.capture(rawCapture, format="bgr")
      image = rawCapture.array
   except:
      print("Failed to capture")
 
   # display the image on screen and wait for a keypress
   try:
      cv2.imshow("Image", image)
      cv2.waitKey(0)
      cv2.destroyAllWindows()
   except:
      print('imshow error')

   # save the image to the disk
   print("Saving image "+fileName)
   try:
      cv2.imwrite(fileName, image)
   except:
      print("Couldn't save "+fileName)
      pass
   
   cv2.waitKey(0)
