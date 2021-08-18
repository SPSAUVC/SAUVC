import serial
import time,threading
import cv2
from ctypes import *
import concurrent.futures

from brping import Ping1D #importing classs Ping1D from brping library
import time #time functions

from builtins import input
import numpy as np#useing numpy for array function

from pysabertooth import Sabertooth

value = 0

error = 0

Set = 0



Kp =7
Ki=0.2
Kd=0.25

Dist = 400


value1 = 0
value2 = 0
value3= 0
value4=0

SpeedNowL = 50
SpeedNowR = 50

cap = cv2.VideoCapture(0)#chooses and opens camera

ser = serial.Serial("/dev/ttyUSB0",9600)

saber=Sabertooth('/dev/ttyS0',baudrate=9600,address=128,timeout=0.1)

myPing = Ping1D() #sets my ping as class Ping1D
myPing.connect_serial("/dev/ttyUSB0", 9600) #connects to the port sonar is at


if myPing.initialize() is False: # this checks for a ping, to make sure sonar working
    print("failed to intialize ping")
    exit(1)

#function to capture and detect green in live feed from camera
def green(frame):
    while True:
            #_ means value returning from a function.
            #_, frame = cap.read(0)
            
            #hsv stands for hue saturation value.
            #Pass every frame value, captured by camera into function
            #".cvtColor" and convert from BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            #[0,0,0] = allow every color
            
            #red = 150,150,0 / 180,255,255
            #blue = 94,80,2 / 120,255,255i
            #green = 25,52,72 / 102,255,255
            
            lower_green = np.array([25,52,72])
            upper_green = np.array([102,255,255])
            
            #mask is everything within lower_red and upper_red
            #mask is either 0 or 1, either within or !within range
            mask = cv2.inRange(hsv, lower_green, upper_green)

            #white takes the white pixels, representing green, from our mask
            white = np.sum(mask == 255)
            print("number of white pixel", white)
            
            
            res = cv2.bitwise_and(frame, frame, mask = mask)
            
            if cv2.waitKey(1) == 27:
                break
            
            return white, res, frame
    
    cv2.destroyAllWindows()
    cap.release()



def get_angle(ser):
    global value
    ser.reset_input_buffer()
    #value=0 # random value to make sure that there is no error when the if statement isn't runninbg
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
                
            cp= pointer(c_int(z)) #makes x into c int
            fp=cast(cp,POINTER(c_float))
            value=fp.contents.value

        
        #print(value)
        #angle=X
           
        if (-180<value) and (value<180):
            return value
        
def sonar():
    data = myPing.get_distance_simple() # gets simple distance data, includes distance and confidence
    Dist = data["distance"]/10# converts distance data from mm to cm
    return Dist


def rescale_frame(frame,scale_percent = 75):# this function is to downscale the camera to reduce the computational power
    width = int(frame.shape[1] * scale_percent /100)
    height = int(frame.shape[0] * scale_percent /100)
    dim = (width,height)
    return cv2.resize(frame,dim,interpolation = cv2.INTER_LINEAR)

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

def pid(error, kp, ki, kd):
    
    global PErr_last
    global IntErr
    
    M_t =0
    PErr_now = (error/360) *100
    DErr = PErr_now - PErr_last
    IntErr += PErr_now
    #print(IntErr)
   

    
    M_t = ( kp * PErr_now + ki * IntErr + kd * DErr)
    
    if M_t>100:
        M_t=100
    if M_t<-100:
        M_t=-100
    
    PErr_last= PErr_now
    
    
    return abs(M_t)


PErr_last = 0 
IntErr = 0
M=0


def FORWARD(m1, m2):
    saber.drive(2,-m1)
    saber.drive(1,-m2)
   #print("m1:",m1)
   #print("m2:",m2)
   #print("forward")
    
def RIGHT_TURN(m1, m2,M):
    new_m1=int(m1*(1.0-(M/100.0)))
    saber.drive(2,-new_m1)
    saber.drive(1,-m2)
    #print(M)
   # print("m1:",new_m1)
    #print("m2:",m2)
    #print("right")

def LEFT_TURN(m1,m2,M):
    new_m2=int(m2*(1.0-(M/100.0)))
    saber.drive(2,-m1)
    saber.drive(1,-new_m2)
    #print(M)
   # print("m1:",m1)
    #print("m2:",new_m2)
    #print("left")
               
def stop1():
    saber.drive(2,0)
    saber.drive(1,0)
    import CompetitionCode_Path4

def stop2():
    saber.drive(2,0)
    saber.drive(1,0)
    import CompetitionCode_Path5
    
    
while True:
    ret, frame = cap.read() # open and read from camera frame
    #this concurrent.futures is the multiprocessing chunk
    with concurrent.futures.ThreadPoolExecutor() as executor:#this is setup to call in the executor function, which is used to multithread
        sona = executor.submit(sonar,)#Executor.submit is used to select and thread the functions we wish to thread
        comp = executor.submit(get_angle, ser)#inside the brackets, specify as follows, executor.submit(insert ur function, insert ur arguments)
        camera = executor.submit(green, frame)
        cam = camera.result()# .result is a class function used to collect the return values from ur functions
        distance = sona.result() 
        now = comp.result()
    if now<0:# now is the compass readings, which is from -180 to 180degree
        now=360+now# this functions changes the reading to follow a standard 360degree compass readings
    error=calculate_error_new(Set,now, error, value1, value2, value3, value4)#this is calling in the error calculation function
    resize = rescale_frame(cam[2])# resizes our camera frame to reduce prcessing power
    error_perr = error[0]/360 * 100# this just changes our error recieved from normal decimals to percentage
    print("Set angle:")
    print(Set)
    print("Angle Now:")
    print(now)
    print("error:")
    print(error[0])
    print("sonar:")
    print(distance)
    print("\n")
    cv2.imshow('frame', cam[2])# showing the frame of our camera for you to see
    
    
    if cam[0] < 17000:
        if 0< error[0] <5 and distance >300 : #initial error threshold is 0 to 5 percent, makes sure our bot is heading in the right direction
            # and that our bot is still far from the specified distance
                    FORWARD(SpeedNowL,SpeedNowR)
                    M=0
                    PErr_last=0
                    IntErr=0
                    print("angle correct")
                    
        elif error[0] > 5:# this is the PID code that will run when our bot goes in the wrong direction
            M=pid(error[0],Kp,Ki,Kd)# this is calling in the PID function
            if error[1] > error[2] or error[3] < error[4]:
                RIGHT_TURN(SpeedNowL,SpeedNowR,M)
                print("right")
            elif error[1] < error[2] or error[3] > error[4]:
                LEFT_TURN(SpeedNowL,SpeedNowR,M)
                print("left")
            elif error[1] == error[2] or error[3] == error[4]:
                RIGHT_TURN(SpeedNowL, SpeedNowR,M)
                print("turn right fully")
        
        elif distance < 300 :
            stop1()
            break
    elif cam[0] >= 17000:
        stop2()
        break
    
cv2.destroyAllWindows()
cap.release()
