import serial
import time #Time functions
import cv2
import numpy as np #Using numpy for array function
from ctypes import *
import math
from pysabertooth import Sabertooth
import RPi.GPIO as GPIO

#Set up for kill switch
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN)
Set = 0
error = 0

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

iCounter = 1
angle_aquistion_bool = False

iTimer = 0

dCounter = 0

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
        
        #Checks whether compass is receiving data.
        if (mydata[0]==0xfa) and (mydata[1] ==0xff):
            #Shifting bytes to make 3 principle axes. X (Roll), Y (Pitch) and Z(Yaw)
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
         
            x = data1 + data2 + data3 + data4 #Roll
            y= data5 + data6 + data7 + data8 #Pitch
            z = data9 + data10 + data11 + data12 #Yaw
                
            cp = pointer(c_int(z)) 
            fp = cast(cp,POINTER(c_float))
            value = fp.contents.value
           
        if (-180<value) and (value<180):
            return value

#This functions calculates the error of the angle we are receiving (Current angle) and our desired angle.
def calculate_error_new(ang_set, ang_now, error, value1, value2, value3, value4):
    
    if ang_set == ang_now:
        #Desired angle equals to current angle.
        error = 0
    
    #If desired angle isn't equals to current angle, we need to find the shortest path to travel
    #to get to the desired angle from the current angle.
    elif ang_set > ang_now:
        #Desired angle is larger than current angle.
        #For example, desired angle is 150 and current angle is 90.
        value3 = 360 - ang_set +ang_now # 300 = 360 - 150 + 90
        value4 = ang_set - ang_now #60 = 150 - 90
        
        #Uses the shortest distance here.
        #Since value4<value3, we should use value4's distance.
        if value4 > value3:
            error = value3 #Error becomes value3.
        elif value4 < value3:
            error = value4
        #Incase we have a 180 difference, we need this condition. For example,
        #Desired angle = 180, traveling from 360.
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

while True:
    #Kill switch
    if GPIO.input(23) == 0:
        break
    
    now=int(get_angle(ser)) #Gets one compass reading
    
    #Sometimes the compass doesn't return any value, we need to be able to display an error message
    #and make sure we don't store that value as part of the compass reading.
    if now == 0:
        print("Compass error")
        
    else:
        #Makes sure angle is correct.
        if now<0:
            now=360+now

        if (now > 0):
            if angle_aquistion_bool == False:
                new = 0
                for iCounter in range (10):
                    now = int (get_angle(ser)) #Gets compass reading
                    #Makes sure angle is correct.
                    if now < 0: 
                        now = 360 + now 
                    #Increment new
                    new = new + now
                    #Display incremented new with the for loop counter.
                    print(str(iCounter + 1) + ": " + str(new))
                else:
                    
                    iCounter = iCounter + 1
                    
                    #Divides new with iCounter to get average compass reading
                    new = new/iCounter
                    new = abs(new)
                    
                    Set = new
                    
                    print("\nAngle aquisition done.")
                    print("Average degree acquired: " + str(Set))
                    print("==========================")
                    
                    #angle_aquistion_bool true to prevent code from repeating above average compass reading loop
                    angle_aquistion_bool = True
                    
                    #Sleep to give the tester time to face robot forward.
                    time.sleep(5)            
        
        #Calculate shortest distance
        error=calculate_error_new(Set,now, error, value1, value2, value3, value4)
    
        #We need to take the 2nd value of 'distance' because if we take the first, some error caused by the compass
        #causes it to reset, you can try to remove the if statement and just take the first value that distance shows.
        if (dCounter < 2):
            
            #Calculates distance required to travel at either the left or right
            distance = 12/math.tan((90-error[0])*(math.pi/180))
            distance = abs(distance)
            
            dCounter = dCounter + 1
        
        else:
#             print("Current angle: " + str(now))
#             print("Set(Desired) angle: " + str(Set))
#             print("Error in degrees: " + str(error[0]) + "\n")
            
#             distance_low = distance * 0.95
#             distance_high = distance * 1.1
#             distance_avg = (distance_low + distance_high)/2
            distance_avg = str(round(distance,5))
            
#             print("The distance required to travel: " + distance_avg)
            
            #Creates a state file and inputs distance.
            f = open("distance.txt", "w+")
            f.write(distance_avg)
            f.close()
            
            g = open("rotation.txt", "w+")
            
            #Depending on whether to turn left or right, create another state file
            #call "rotation".
            
            if error[1] < error[2] or error[3] > error[4]:
                g.write("turn left")
                g.close()
            
            elif error[1] == error[2] or error[3] == error[4]:
                g.write("turn right")
                g.close()
    
            #Call the second part of the code.
            import CompetitionCode_Path1_part2.py
            
            #Once done, break code.
            break
              
    if cv2.waitKey(1) == 27:
        break
