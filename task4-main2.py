# Team ID : 2361
# Author List : K Pranath Reddy
# Filename : task4-main2.py
# Theme : Harvestor Bot
# Functions : NONE
# Global variables : cam,img,image,gray,blur,circles,k,x,X,y,Y,r,R,B,G,RED,blue,green,red,Area,ser

# importing the required modules
import cv2
import numpy as np
from math import pi
import pygame
import pygame.camera
import time
import task4-main.py
import serial

while True :

    # while loop runs the code continously
    # we use try and except for error handling
    try :
        ser=serial.Serial('/dev/ttyUSB0',9600,timeout=0)

        # the below lines of code takes image from the camera and that image is used as the input for using hough circles to detect the fruits 
        pygame.camera.init()
        pygame.camera.list_cameras()
        cam = pygame.camera.Camera("/dev/video0", (640, 480))
        cam.start()
        time.sleep(0.1)  # You might need something higher in the beginning
        # printing message
        print("taking image")
        img = cam.get_image()
        pygame.image.save(img, "pygame.png")
        # the above line saves the input image  
        cam.stop()
        # Stops the camera
        
        # printing message
        print("image processing start")

        # Reading the input image obtained 
        image = cv2.imread('pygame.png')
        
        # convert the input image into grayscale and inserts a blur filter 
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
      
        # We approximate the shape of the fruit to be a circle

        # detecting circles in the image using the inbuilt houghcircles function of openCV
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1.5, 10)
        circles = np.uint16(np.around(circles))

        # Counting the number of circles obtained in the image
        k=0
        for i in circles[0,:]:
            k = k+1
        
        # finding the average of the x-coordinates of the centre of all the circles obtained in the image 
        x = 0
        # x - individual x-coordinates
        for a in range(0,k):
            x = x + circles[0][a][0]
        X = x/k
        # X - average value of x-coordinates 

        # finding the average of all the y-coordinates of the centre of all the circles obtained in the image
        y = 0
        # y - individual y-coordinates
        for b in range(0,k):
            y = y + circles[0][b][1]
        Y =y/k
        # Y - average value of y-coordinates

        # Finding the average of the radius of all the circles obtained
        r = 0
        # r - individual radius values
        for c in range(0,k):
            r = r + circles[0][c][2] 
        R =r/k
        # R - average radius value

        # Finding the average of the blue color value of all the points along the radius
        blue = 0
        # blue - individual blue color values
        for d in range(0,R):
            blue = blue + image[Y,X+d,0]
        B = blue/R
        # B - average blue color value

        # Finding the average of the green color value of all the points along the radius
        green = 0
        # green - individual green color value
        for e in range(0,R):
            green = green + image[Y,X+e,1]
        G = green/R
        # G - average green color value

        # Finding the average value of the red color value of all the points along the radius
        red = 0
        # red - individual red color values
        for f in range(0,R):
            red = red + image[Y,X+f,2]
        RED = red/R
        # RED - average red color value

        # We are using color values to find the type of fruit in the input frame

        # Area of the circle (fruit)
        Area =  int(pi * R**2)
        # We are using area to find the size of the fruit

        # can be used to print the circle in the output image
        cv2.circle(image,(X,Y),R,(0,0,255),2)

        #The below if statemants have the area ranges and color ranges of various fruits
        if(Area<7000):
            if(B>150):
                print("small blueberry")
                ser.write(1)
            elif(G>120):
                print("small orange")
                ser.write(2)
            else:
                print("small apple")
                ser.write(3)

        if(12000<Area<17000):
            if(B>150):
                print("medium blueberry")
                ser.write(4)
            elif(G>120):
                print("medium orange")
                ser.write(5)
            else:
                print("medium apple")
                ser.write(6)

        if(22000<Area):
            if(B>150):
                print("Large blueberry")
                ser.write(7)
            elif(G>120):
                print("Large orange")
                ser.write(8)
            else:
                print("Large apple")
                ser.write(9)

        # the below lines can be used to print the area and color of the fruit obtained
        # print("Area of the fruit is" + " " + str(Area))
        # print("Color of the fruit is (" + str(B) + "," + str(G) + "," + str(RED) + ")")

        # the below lines can be used to display the output image
        # cv2.imshow('detected circles', image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        ser.close
        time.sleep(2) 
        # code stops for 2 seconds so that the bot an pick the fruit and turn away from the tree

        task4-main 
        # opens code_loop_final after the above time runs out
        
    except :

        task4-main 
        # opens code_loop_final if error occurs

# We are using two python files to run the code continously 

