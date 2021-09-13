#This code has not been verified but I tried my best to get
#the best product I could while working at home. Do make sure
#to test this code at the pool.

#This code essentially does this few things:
#
# 1) Check gate position relative to vehicle
# 2) Position vehicle correctly while double checking
# 3) Move forward to enter gate
# 4) Stop vehicle
# 5) Import next code

import numpy as np
import cv2
import time
import serial
from ctypes import *
from pysabertooth import Sabertooth
from brping import PingID 

object_detector = cv2.createBackgroundSubtractorMOG2(history = 100, varThreshold = 40)

webcam = cv2.VideoCapture(0) #Open camera, use either '-1', '0' or '1'

Set = 0
error = 0

desire_angle_addition_int = 0 
desire_angle_addition_Bool = True

Kp =7
Ki=0.2
Kd=0.25

value = 0
value1 = 0
value2 = 0
value3 = 0
value4 = 0

SpeedNowL = 65
SpeedNowR = 65

i = 0
ser = serial.Serial("/dev/ttyUSB0",9600)

myPing = pingID()
myPing.connect_serial("/dev/ttyUSB1", 9600)

saber=Sabertooth('/dev/ttyS0',baudrate=9600,address=128,timeout=0.1)

gate_right = False
gate_left = False

#You won't be viewing anything during competition, but this isn't for you.
width = 640
height = 640
dim = (width, height)

#Checks for a ping, to make sure sonar is working
if myPing.initialize() is False:
    print("Failed to initialize ping")

def sonar():
    data = myPing.get_distance_simple() #Gets simple distance data, including distance
    Dist = data["distance"]/10 #Converts distance data from mm to cm
    return Dist

def get_angle(ser):
    global value
    ser.reset_input_buffer()
    mydata=[]
    while True:
        count=0
        done =1
        while done !=0:
            for count in range(0,20):
                data = ser.read()
                data=hex(ord(data)) 
                data=int(data,16)
                mydata.append(data)
            count=0
            done=0
                
        if (mydata[0]==0xfa) and (mydata[1] ==0xff):

            data1=mydata[7]<<24
            data2=mydata[8]<<16
            data3=mydata[9]<<8
            data4=mydata[10]
            data5=mydata[11]<<24
            data6=mydata[12]<<16
            data7=mydata[13]<<8
            data8=mydata[14]
            data9=mydata[15]<<24
            data10=mydata[16]<<16
            data11=mydata[17]<<8
            data12=mydata[18]
         
            x = data1 + data2 + data3 + data4
            y= data5 + data6 + data7 + data8
            z = data9 + data10 + data11 + data12
                
            cp= pointer(c_int(z)) 
            fp=cast(cp,POINTER(c_float))
            value = fp.contents.value
           
        if (-180<value) and (value<180):
            return value

        
def calculate_error_new(ang_set, ang_now, error, value1, value2, value3, value4):
    if ang_set == ang_now:
        error = 0
    elif ang_set > ang_now:
        value3 = 360- ang_set +ang_now
        value4 = ang_set - ang_now
        
        if value4 > value3:
            error = value3
        elif value4 < value3:
            error = value4
        elif  value4 == value3:
            error = value4
            
    elif ang_set < ang_now:
        value1 = 360 - ang_now +ang_set
        value2 = ang_now - ang_set
        
        if value1 > value2:
            error = value2
        elif value1< value2:
            error = value1
        elif value1 == value2:
            error = value1
            
    return error, value1, value2, value3, value4


#Function to run PID calculation. Using error calculated and set kp,ki,kd constants,
#this code will run and calculate the variable M_t which will be used
#to calculate the change in speed needed for the motors to help our bot turn left or right
#to fix our error and return it back on track moving forwards in the direction of our set angle. 
def pid(error, kp, ki, kd):
    
    global PErr_last
    global IntErr
    
    M_t =0
    PErr_now = (error/360) *100# calculation to get our proportional error
    DErr = PErr_now - PErr_last# calculation to get our differential error
    IntErr += PErr_now# calculation to get our inegral error

    M_t = ( kp * PErr_now + ki * IntErr + kd * DErr) # calculation to get our percentage change in speed needed to fix error
    
    # if statement to cap our percentage change at 100%
    if M_t>100:
        M_t=100
    if M_t<-100:
        M_t=-100
    
    PErr_last= PErr_now
    
    return abs(M_t)

#Functions to drive AUV in respective direction
def FORWARD(m1, m2):
    saber.drive(2,-m1)
    saber.drive(1,-m2)
    
    print("m1:",m1)
    print("m2:",m2)
    print("Moving forward\n")
    print("=================")
    
    
def RIGHT_TURN(m1, m2,M):
    new_m1=int(m1*(1.0-(M/100.0)))
    saber.drive(2,-new_m1)
    saber.drive(1,-m2)
    
    print("m1:",new_m1)
    print("m2:",m2)
    print("Turning right")
    print("=================")

def LEFT_TURN(m1,m2,M):
    new_m2=int(m2*(1.0-(M/100.0)))
    saber.drive(2,-m1)
    saber.drive(1,-new_m2)
    
    print("m1:",m1)
    print("m2:",new_m2)
    print("Turning left")
    print("=================")
               
def stop():
    saber.drive(1,0)
    saber.drive(2,0)
    
    print("m1: 0")
    print("m2: 0")
    print("Stopping\n")
    
    f = open("state.txt", "w")
    f.write("2")
    f.close()
   
PErr_last = 0 
IntErr = 0
M=0   
desire_angle_addition_Bool = False

while True:
    _, frame = webcam.read() #Read frame
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA) #Resizes frame
    
    frame_copy = np.copy(frame) #Copies frame
    
    kernel = 1
    frame = cv2.GaussianBlur(frame, (kernel, kernel), cv2.IMREAD_UNCHANGED)
    
    #'hsv' value for masking gate on a sunny day is roughly:
    #
    # hue: 104-145
    # sat: 0-255
    # value: 0-255
    
    #Do note that during competition/testing the values can be completedly different
    
    #Masking away value
    lowerRegion = np.array([104,0,0],np.uint8)
    upperRegion = np.array([145,255,255],np.uint8)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    redObject = cv2.inRange(hsv,lowerRegion,upperRegion)
    
    kernel = np.ones((1,1),"uint8")
    red = cv2.morphologyEx(redObject,cv2.MORPH_OPEN,kernel)
    red = cv2.dilate(red,kernel,iterations=1)

    res1 = cv2.bitwise_and(frame, frame, mask = red)
    
    #Determine where 'res1' is
    mask2 = object_detector.apply(res1)
    _, mask2 = cv2.threshold(mask2, 254, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        #This contour size, 'cntSize', only works for 640x640 frame size, DO NOT change this.
        cntSize = 900
        if area > cntSize and area < (cntSize + 20):
            
            #cv2.drawContours(res1, [cnt], -1, (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(cnt)
            
            #Check whether such co-ordinates 'X, Y, W, H' is a tall slim vertical rectangle
            #The values of 15, 20, 185 and 190 are all assumed value. They don't hold any
            #meaning. I got those number by seeing the pattern when I print the dimensions
            #of the gate vs when I print the dimensions of noise.

            x1 = x
            y1 = y
            x2 = x + w
            y2 = y + h
            width = w
            height = h
            
            #Checks width
            if width > 15 and width < 20:
                
                #Checks height
                if height > 185 and height < 190:
                    
                    cv2.rectangle(res1, (x1,y1), (x2, y2), (0, 255, 0), 3)
                    print("Gate detected at co-ordinates: ")
                    print("x1: " + str(x1))
                    print("x2: " + str(x2))
                    print("y1: " + str(y1))
                    print("y2: " + str(y2))
                    print("\n")

                    #Since we know that the pole on the left/right is 1.2m tall
                    #and the height of y1 to y2 is also the same. We can
                    #deduce that y2-y1/'height' = 1.2m.
                    #Furthermore the thickness of the pole is also, 'width'.
                    
                    #So with our theory:
                    # 40cm/0.4m = x2-x1 / 'width'
                    # 120cm/1.2m = y2-y1 / 'height
                    
                    metre_pixels_cm_width = (width*2.353/40) #Pixels for 1cm. I multiply here by 2.353 because I needed a tolerance. As the value wasn't right.
                    metre_pixels_cm_height = height/120
                    
                    #We need to build the gate now (1.5m x 1.2m)
                    gate_length = metre_pixels_cm_width * 150
                    x1_gate = x2 - gate_length
                    
                    gate_height = metre_pixels_cm_height * 120
                    y1_gate = y2 - gate_height
                    
                    #cv2.rectangle(res1, (int(x1_gate), int(y1_gate)), (int(x2), int(y2)), (0,255, 0), 3)
                    
                    # Draw circle in center
                    xcenter = x1_gate + (int(round((x2 - x1_gate) / 2)))
                    ycenter = y1_gate + (int(round((y2 - y1_gate) / 2)))
                    #cv2.circle(res1, (int(xcenter), int(ycenter)), 5, (0,0,255), thickness=-1)
                    
                    #Determine where circle is (x-axis) on it's y-axis.
                    #Since we resized the frame to 640, we know that the middle
                    #is 640.
                    
                    middle_point = 640/2
                    tolerance = 5
                    middle_point_acceptable_min = middle_point - 640/tolerance
                    middle_point_acceptable_max = middle_point + 640/tolerance
                    
                    if xcenter > middle_point_acceptable_min and xcenter < middle_point_acceptable_max:
                        #Gate is within acceptable range of 45% - 55% of screen.
                        #Go forward
                        
                        #print("Heading forward")
                        
                        distance = sonar()
                        while distance > 1200:
                            M=0
                            PErr_last=0
                            IntErr=0
                            FORWARD(SpeedNowL,SpeedNowR)
                      
                        stop()
                        break
                        
                    elif xcenter > middle_point:
                        
                        #Gate is to the right
                        #Determine the degree to turn
                        
                        #Since we already know the how many pixels is per cm
                        #we will just use the distance between the circle/center of gate
                        #with where the vehicle is pointing to calculate the distance using trigo
                        distance_1 = (xcenter-middle_point)/metre_pixels_cm_width
                        
                        #Using trigo to find the required degree to turn
                        #
                        #adjacent = ~2.5m/vehicle to gate
                        #hypotenuse = unknown
                        #opposite = 'distance_1'
                        
                        #o & a use tanjent
                        #python is normally radidan mode
                        degree_1 = math.atan(2.5/distance_1)*(180/math.pi)
                        
                        gate_right = True
                        
                        
                    elif xcenter < middle_point:
                        #Gate is to the left
                        #Determine the degree to turn
                        
                        distance_2 = (middle_point - xcenter)/metre_pixels_cm_width
                        degree_2 = math.atan(2.5/distance_1)*(180/math.pi)
                        
                        gate_left = True
                    
                    while True:
                        if gate_right or gate_left is True:
                            
                            
                            #Get angle
                            now=int(get_angle(ser))
            
                            if now<0:
                                now=360+now
                            
                            if (now > 0 and desire_angle_addition_Bool == False):
                                if gate_right is True:
                                    Set = now + desire_angle_addition_int
                                elif gate_left is True:
                                    Set = now - desire_angle_addition_int
                            
                            if (Set > 360):
                                Set = Set - 360
                                desire_angle_addition_Bool = True
                            
                            desire_angle_addition_Bool = True
                        
                        error=calculate_error_new(Set,now, error, value1, value2, value3, value4)
                            
                        print("Current angle: " + str(now))
                        print("Set(Desired) angle: " + str(Set))
                        print("Error in degrees: " + str(error[0]) + "\n")
                        
                        if (0< error[0] <5):
                            distance = sonar()
                            
                        while distance > 1200:
                            M=0
                            PErr_last=0
                            IntErr=0
                            FORWARD(SpeedNowL,SpeedNowR)
                      
                        stop()
                        break
                        
                        else:
                            M=pid(error[0],Kp,Ki,Kd)
                            if error[1] > error[2] or error[3] < error[4]:# error[1] = value1, error[2]=value2, error[3]=check(can be true or false)
                                RIGHT_TURN(SpeedNowL,SpeedNowR,M)
                                
                            elif error[1] < error[2] or error[3] > error[4] :
                                LEFT_TURN(SpeedNowL,SpeedNowR,M)
                                
                            elif error[1] == error[2] or error[3] == error[4]:
                                RIGHT_TURN(SpeedNowL,SpeedNowR,M)
              
    #Wait for key to destroy
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break 

#Destorys all windows and closes webcamera
cv2.destroyAllWindows()
webcam.release()

    




