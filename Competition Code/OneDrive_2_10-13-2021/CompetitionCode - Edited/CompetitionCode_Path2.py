import serial# Library for serial functions
import time # Library for time functions
import cv2 # Opencv Library
from ctypes import * # Library for c type calculations and conversions
import concurrent.futures # Library for multithreading

from brping import Ping1D #importing classs Ping1D from brping library
import time #time functions

from builtins import input
import numpy as np#useing numpy for array function

from pysabertooth import Sabertooth # Library to help control sabertooth

value = 0



error = 0

Set = 188 # compass angle we set

Kp =7# constants for PID
Ki=0.2
Kd=0.25


value1 = 0
value2 = 0
value3= 0
value4=0

SpeedNowL = 50 # speed used for both motors when going forwards
SpeedNowR = 50

ser = serial.Serial("/dev/ttyUSB0",9600)# setting ser object at serial port USB0, where compass sends data to

saber=Sabertooth('/dev/ttyS0',baudrate=9600,address=128,timeout=0.1) # sets saber object as class sabertooth controlling
                                                                        #the horizontal sabertooth

myPing = Ping1D() #sets my ping as class Ping1D
myPing.connect_serial("/dev/ttyUSB1", 9600) #connects to the port sonar is at


if myPing.initialize() is False: # this checks for a ping, to make sure sonar working
    print("failed to intialize ping")
    exit(1)


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
                #print(data)
                data=hex(ord(data)) #gives in str
                #print (data)
                data=int(data,16)
                mydata.append(data)
            count=0
            done=0
            #print(mydata[0],mydata[1])
        #if statement to make sure the data is recieving compass        
        if (mydata[0]==0xfa) and (mydata[1] ==0xff):
            #shifting bytes to make 3 principle axis, x, y and z, which is just roll, pitch and yaw
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
         
            x = data1 + data2 + data3 + data4# roll data
            y= data5 + data6 + data7 + data8# pitch data
            z = data9 + data10 + data11 + data12# yaw data, this is the one we need
                
            cp= pointer(c_int(z)) #makes Z into c int
            fp=cast(cp,POINTER(c_float))
            value=fp.contents.value

        
           
        if (-180<value) and (value<180):
            return value
#This function is to recieve distance data from our sonar and convert the recieved data into centermetres      
def sonar():
    data = myPing.get_distance_simple() # gets simple distance data, includes distance and confidence
    Dist = data["distance"]/10# stores and converts distance data from mm to cm into object Dist
    return Dist


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


#Function to run PID calculation. Using error calculated and set kp,ki,kd constants, this code will run and calculate the variable M_t which will be used
#to calculate the change in speed needed for the motors to help our bot turn left or right to fix our error and return it back on track moving forwards in the direction of our set angle. 
def pid(error, kp, ki, kd):
    
    global PErr_last
    global IntErr
    
    M_t =0
    PErr_now = (error/360) *100# calculation to get our proportional error
    DErr = PErr_now - PErr_last# calculation to get our differential error
    IntErr += PErr_now# calculation to get our inegral error
   

    
    M_t = ( kp * PErr_now + ki * IntErr + kd * DErr)# calculation to get our percentage change in speed needed to fix error
    
    if M_t>100:# if statement to cap our percentage change at 100%
        M_t=100
    if M_t<-100:
        M_t=-100
    
    PErr_last= PErr_now
    
    
    return abs(M_t)


PErr_last = 0 
IntErr = 0
M=0


def FORWARD(m1, m2):# function to drive AUV forwards
    saber.drive(2,-m1)
    saber.drive(1,-m2)
   #print("m1:",m1)
   #print("m2:",m2)
   #print("forward")
    
def RIGHT_TURN(m1, m2,M):# function to turn AUV right
    new_m1=int(m1*(1.0-(M/100.0)))
    saber.drive(2,-new_m1)
    saber.drive(1,-m2)
    #print(M)
   # print("m1:",new_m1)
    #print("m2:",m2)
    #print("right")

def LEFT_TURN(m1,m2,M):
    new_m2=int(m2*(1.0-(M/100.0)))#function to turn AUV left
    saber.drive(2,-m1)
    saber.drive(1,-new_m2)
    #print(M)
   # print("m1:",m1)
    #print("m2:",new_m2)
    #print("left")
               
def stop():# function to stop AUV and run next path code
    saber.drive(2,0)
    saber.drive(1,0)
    import CompetitionCode_Path3





while True:
    # Multi-threading to run compass and sonar at the same time
    #Multi-threading basically lets us run functions or loops in parallel, but this does not mean they  
    with concurrent.futures.ThreadPoolExecutor() as executor:
        sona = executor.submit(sonar,)# class function to thread sonar function
        comp = executor.submit(get_angle, ser)# class function to thread compass function
        distance = sona.result()# class function to recieve and store return value from sonar function
        now = comp.result()#class function to recieve and store return values from compass function
    if now<0:
        now=360+now# make our angle work in the 360 degree spectrum
    error=calculate_error_new(Set,now, error, value1, value2, value3, value4)# store our error calculation error into object error
    error_perr = error[0]/360 * 100# convert our error to percentage
    print("Set angle:")
    print(Set)
    print("Angle Now:")
    print(now)
    print("error:")
    print(error[0])
    print("sonar:")
    print(distance)
    print("\n")
    
    #this whole chunk of code is the main checks to help our AUV determine if its reached its destination, which is where the buckets will be
    if 0< error[0] <5 and distance >400 :#if statement to run to going straight to reach our destination
                FORWARD(SpeedNowL,SpeedNowR)# function to run our motors to go forward
                M=0# resetting our PID M
                PErr_last=0# resetting our proportional error
                IntErr=0# resetting our integral error
                print("angle correct")
                
    elif error[0] > 5:# if statement to run for when AUV drives off course
        M=pid(error[0],Kp,Ki,Kd)# M_t for motor speed calculation
        if error[1] > error[2] or error[3] < error[4]:# if statement to run if our AUV should turn right
            RIGHT_TURN(SpeedNowL,SpeedNowR,M)# function to turn right
            print("right")
        elif error[1] < error[2] or error[3] > error[4]:# if statement to run when our AUV should turn left
            LEFT_TURN(SpeedNowL,SpeedNowR,M)# function to turn left
            print("left")
        elif error[1] == error[2] or error[3] == error[4]:#if statement to run when left or right are both the shortest path
            RIGHT_TURN(SpeedNowL, SpeedNowR,M)# function to turn right(this is for when either left or right both the shortest path, so we chose right)
            print("turn right fully")
    
    elif distance < 400 :# if statement to run when our destination is reached to kill this path code and run enxt path code
        stop()
        break



       
    
