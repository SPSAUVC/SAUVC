def get_angle2(): #Getting compass value.
    mydata=[]
    count=0
    done =1
    while done !=0:
        if ser.inWaiting!=0:
            for count in range(0,20):
                data = ser.read()
                #'data' returns as unicode code point value of 'data'
                # and converts into hexadecimal value.
                data=hex(ord(data))
                data=int(data,16) #Convert 'data' (hexadecimal) to integer.
                mydata.append(data)
        count=0
        done=0
   
    # First 2 bytes will always be '0xfa' and '0xff'.
    # Checking, will always be true unless no bytes coming true.
    if (mydata[0]==0xfa) and (mydata[1] == 0xff):
        #Row
        data1=mydata[7]<<24
        data2=mydata[8]<<16
        data3=mydata[9]<<8
        data4=mydata[10]
        #Pitch
        data5=mydata[11]<<24
        data6=mydata[12]<<16
        data7=mydata[13]<<8
        data8=mydata[14]
        #Yaw
        data9=mydata[15]<<24
        data10=mydata[16]<<16
        data11=mydata[17]<<8
        data12=mydata[18]
     
        #Combining the value to get x,y and z values.
        x = data1 + data2 + data3 + data4
        y= data5 + data6 + data7 + data8
        z = data9 + data10 + data11 + data12
            
        cp= pointer(c_int(z)) #Convert 'z' into c_int
        fp=cast(cp,POINTER(c_float))
        H=fp.contents.value
            
        xp= pointer(c_int(x)) #Convert 'x' into c int
        tp=cast(xp,POINTER(c_float))
        X=tp.contents.value
        
        X=int(X)
        angle1=str(X)
            
        f=open("angle.txt","a") #Save x value into angle.txt file'
        f.write("x value:")
        f.write(angle1)
        f.write("\n")
        f.close()

    value=H
    #angle=X
        
    if (-180<H) and (H<180):
        return value;
        
    
def calculate_error(ang_set, ang_now): #Error here is how far of the bot is from the desired angle.

    error=0
    right_indx=[]
    left_indx=[]
    found=0
    
    #Right scale
    for i in range (0,181): #Count from 0 - 180, 181 digits
        #All the values in the right scale get saved into this right index.
        value=ang_set + i
        if (value > 359):
            value = value - 360
        right_indx.append(value)
        
    right_min = ang_set
    right_max=value
    
    #Left scale
    for i in range(0,179): #Count from 0 - 178, 179 digits
        #All the values in the left scale get saved into this left index.
        value = ang_set - i -1
        if value <0:
            value=value+360
        left_indx.append(value)
        
    if left_min<0:
        left_min = left_min +360
        
    left_min = ang_set -1
    left_max =value

    #Find positive or negative error.
    #Positive, angle set right hand up to 180 degree increment. If angle_set is 10, 11 to 190 is positive error.
    #Negative, angle set left hand up to 179 degrees increment. If angle_set is 10, 9 to 191 is negative error.
    if ang_set == ang_now:
        error = 0 #No error.
    else:
        for i in range(0,179):
            if (right_indx[i] == ang_now):
                error=i
                
        for i in range(0,179):
            if (left_indx[i] == ang_now):
                error = i*-1
    return error

def pid(error, kp, ki, kd):
    
    global PErr_last
    global IntErr
    
    M_t =0 #Value for output
    PErr_now = (error/360) *100 #Find error in terms of percentage
    DErr = PErr_now - PErr_last #Derivative term in percentage
    IntErr += PErr_now #Integeral term in percentage
    
    #Compute PID output
    M_t = ( kp * PErr_now + ki * IntErr + kd * DErr)
    
    #Clip the output
    if M_t>100:
        M_t=100 
    if M_t<-100:
        M_t=-100
        
    PErr_last= PErr_now
    return abs(M_t)

def FORWARD(m1, m2):
    saber.drive(2,-m1)
    saber.drive(1,-m2)
    
def RIGHT_TURN(m1, m2):
    new_m1=int(m1*(1.0-(M/100.0)))
    saber.drive(2,-new_m1)
    saber.drive(1,-m2)

def LEFT_TURN(m1,m2):
    new_m2=int(m2*(1.0-(M/100.0)))
    saber.drive(2,-m1)
    saber.drive(1,-new_m2)
               
def stop():
    saber.drive(1,0)
    saber.drive(2,0)
    
def loop(): #main part of the compass code
global Set
Kp =7
Ki=0.2
Kd=0.25
SpeedNowL=0
SpeedNowR=0
data=myPing.get_distance() #doing this to avoid any error with there not being any data[distance]
distance=3000 #this is the starting distance between the bot and the wall which is close to the bucket
             #the bot will do a horizontal scan once and after that this distance will increase, so that
            #the bot can move backwards and scan the pool till it finds the bucket
angles=[]


while True:
    file3=open("compass.txt","r")
    value=file3.readline()
    
    
   # if GPIO.input(23)==0:
     #   saber.drive(1,0)
     #   saber.drive(2,0)
      #  break       
    
    if value=="1":
        sem.acquire()
        lower_distance=distance-1000 #lower threshold for the distance
        angle0=get_angle2()#getting the angle from the compass
        angle1=get_angle2()
        angle2=get_angle2()
        now=int(angle1)
        print("Distance: %s\tConfidence: %s" % (data["distance"], data["confidence"]))
        #print(now)
        if now<0:
            now=359+now
        error=calculate_error(Set,now)
        SpeedNowL=SpeedL()
        #print(SpeedNowL)
        SpeedNowR=SpeedR()
        #print(SpeedNowR)
        print("Set angle:")
        print(Set)
        print("Angle Now:")
        print(now)
        print("error:")
        print(error)
        print("\n")
                
        if (abs(error)<5): # if error is less than 2, it will just go straight
            FORWARD(SpeedNowL,SpeedNowR) # basically speednowl is your m1 and speednowR is your m2
            M=0
            PErr_last=0
            IntErr=0
        else: #else,recompute the PID
            M=pid(error,Kp,Ki,Kd)
        if error>5: #right turn
            RIGHT_TURN(SpeedNowL,SpeedNowR)
        elif(error<-5): #left turn 
            LEFT_TURN(SpeedNowL,SpeedNowR)
        else: #if not right turn, not left turn, then has to be go forward
            FORWARD(SpeedNowL,SpeedNowR)     
            
        if (-5<now<5 and lower_distance<=data["distance"]<=distance): #using the variable distance because the value of the distance will increase slowly till we are able to detect the bucket
            Set=90
            SpeedNowL=50 # Speed is positive allowing it to move forward at the right direction
            SpeedNowR=50 # I lowered the speed so that the sonar will be more accurate
        
        if (85<now<90 and data["distance"]<=500):
            Set=-90
        
        if (-95<now<-85 and data["distance"]<=500):
             Set=0
             SpeedNowL=-80 # the speed is reversed so that the bot can go backwards
             SpeedNowR=-80 # it's going backward so that the bot can detect the wall 
             distance=distance+800
        sem.release()
