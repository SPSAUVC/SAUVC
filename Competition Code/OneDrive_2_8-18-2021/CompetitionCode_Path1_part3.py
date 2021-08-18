import numpy as np
import cv2
import time
import serial
from ctypes import *
from pysabertooth import Sabertooth

Set = 0
error = 0

desire_angle_addition_int = 90 #Change this to desired angle addition
desire_angle_addition_Bool = True

Kp =7
Ki=0.2
Kd=0.25

value = 0
value1 = 0
value2 = 0
value3 = 0
value4 = 0

SpeedNowL = 50
SpeedNowR = 50

i = 0
ser = serial.Serial("/dev/ttyUSB0",9600)
saber=Sabertooth('/dev/ttyS0',baudrate=9600,address=128,timeout=0.1)

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
    now=int(get_angle(ser))
        
    if now<0:
        now=360+now
        
    #I realize that everytime the robot is placed into the water, the angle_now would be different.
    #In order to fix that problem, I devise a code to make sure that everytime we place the robot into
    #the water, no matter the angle_now. It would always +(desired angle).
    
    #But the good part is that it only works once, thats why we need a boolean. If not the robot's angle
    #would continuously add (desired angle) and it will keep turning left.
    if (now > 0 and desire_angle_addition_Bool == False):
        g = open("rotation.txt", "r")
                
        if (g.mode == 'r'):

            rotation_txtfile = g.read()
            print(rotation_txtfile)
            
            #Checks whether turn left or right.
            if "turn left" in rotation_txtfile:
                Set = now - desire_angle_addition_int
                print("Gate @ left")
                
            elif "turn right" in rotation_txtfile:
                Set = now + desire_angle_addition_int
                print("Gate @ right")
                
        if (Set > 360):
            Set = Set - 360
            desire_angle_addition_Bool = True
        
        desire_angle_addition_Bool = True
    
    error=calculate_error_new(Set,now, error, value1, value2, value3, value4)
    
    
    print("Current angle: " + str(now))
    print("Set(Desired) angle: " + str(Set))
    print("Error in degrees: " + str(error[0]) + "\n")
    
    if (0< error[0] <5):
        #Both motors run 50 speed can travel 18m/66seconds = 0.2727m/s
        #Both motors run 100 speed can travel 18m/33seconds = 0.5454m/s
        #Standard competition distance = 12m, if you are testing in the lab and are using a shorter value
        #like 1.5m, please change the respective values from the other files.
        desired_time = 100
        #I used desired_time as 100 because I want the AUV to run for 50 seconds (100/2) seconds you can use mathematical
        #calculations to take 12m (Set distance between wall and gate) divide by how fast the AUV can run m/s.
            
        M=0
        PErr_last=0
        IntErr=0
        FORWARD(SpeedNowL,SpeedNowR)
        
        i = i + 1
        time = i/2
        print(str(time) + "/" + str(desired_time) + " seconds" + "\n")
        
        #Change time here to stop
        #Every i == 2 equals 1 second
        
        if (time > desired_time):
            stop()
            print("Gate reached!")
            break
            
    else:
        M=pid(error[0],Kp,Ki,Kd)
        if error[1] > error[2] or error[3] < error[4]:# error[1] = value1, error[2]=value2, error[3]=check(can be true or false)
            RIGHT_TURN(SpeedNowL,SpeedNowR,M)
            
        elif error[1] < error[2] or error[3] > error[4] :
            LEFT_TURN(SpeedNowL,SpeedNowR,M)
            
        elif error[1] == error[2] or error[3] == error[4]:
            RIGHT_TURN(SpeedNowL,SpeedNowR,M)
           
            
       
    if cv2.waitKey(1) == 27:
        break


