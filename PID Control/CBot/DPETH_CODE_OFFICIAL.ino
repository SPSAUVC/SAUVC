#include <TimerOne.h>// Library same as TimerOne, not neeon
#include <Wire.h> // library to allow for I2C and/or TWI communiation to devices
#include "MS5837.h" //library to configure depth sensor
MS5837 sensor; //setting depth sensor as variable sensor
#include <SoftwareSerial.h> // library for serial communications
#include <SabertoothSimplified.h> // library to send commands to sabertooth to run thrusters
#define setvalue 127; // speed for downwards motion
//SoftwareSerial SWSerial(NOT_A_PIN,11); // sabertooth Rx serial recieve pin
SabertoothSimplified ST(Serial1);  // setting up sabertooth
int DEPTH[3]; // DEPTH array to find median value for finer depth calculations
String D = "72"; // D used to control on off of depth code, D = 'H' will run the code
//String D1="0"; //ignore this for now
int Set_depth = 80; // our codes chosen depth
int MotorSpd1 = -40; // upwards motion motor speeds
int MotorSpd2 = setvalue; // downwards motion motor speed
double error = 0; //for error calculation and PID

double kp = 0.9; //0.8, proportional constant of PID calculations
double ki = 0.02; //0.02, intergral constant of PID calculations
double kd = 0.025; //0.025, derivative constant of PID calculations

double PErr_now = 0; // porpotional error calculation variable and error at the time
double DErr = 0; // derivative error calculation variable

unsigned long lastTime;
double PErr_last = 0; // porpotional error of last 1 sec compared to porptional error now
double IntErr = 0; // intergral error calculation variable
double M = 0; // motor speed control needed to correct error

void setup() {
  Serial.begin(9600); //initialzie serial for arduino
  Wire.begin(); // initialize I2C
  sensor.init(); // initalize dpeth sensor
  sensor.setModel(MS5837::MS5837_30BA); //changed 02BA to 30BA, changing to our new depth sensor
  sensor.setFluidDensity(997); // sensor fluid density sensing setting
  Serial1.begin(9600); // initiate our sabertooth serial
  Timer1.initialize(50000);//100000 from 10000(microseconds), to run GODOWN code after every 0.1secs1
  Timer1.attachInterrupt(GODOWN);// interrupt to create sub-loop
}

void loop() {
  if (Serial.available() != 0) { // checking for serial
    D = Serial.read(); //reading for '72' to come in from RPI
  }
  //This part is to calculate median depth so we can better accurately determine
  //how off our depth is
  int done = 1;
  int x;
  sensor.read();//depths sensor reads in meters
  DEPTH[0] = sensor.depth() * 100; //eg. 0.15 x 100 = 15, this changes it to cm
  sensor.read();
  DEPTH[1] = sensor.depth() * 100;
  sensor.read();
  DEPTH[2] = sensor.depth() * 100;
  //this part still figuring out, will list in journal once ik
  while (done != 0) {
    for (int j = 0; j < 2; j++) {
      if (DEPTH[j] > DEPTH[j + 1]) {
        x = DEPTH[j + 1];
        DEPTH[j + 1] = DEPTH[j];
        DEPTH[j] = x;
        done = 1;
      }
      else done = 0;
    }
  }
  if (DEPTH[1] < 0) {
    DEPTH[1] = 0;
  }
  Serial.println(M);// Motor change need, found in GODOWN
  Serial.println(error);// error from set_depth, found in GODOWN
  Serial.print("DEPTH=");
  Serial.println(DEPTH[1]);// current depth
  Serial.print("PErrMS2=");
  Serial.println(MotorSpd2);// current speed
  Serial.println("\n");
  //delay(1000);// delay to print out properly in serial monitor for reading
}

void GODOWN() {
  if (D == "72") {
    //Set-depth = 60,
    error = double(Set_depth) - double(DEPTH[1]); // error calculation
    PErr_now = (error / 200) * 100; // porpotional error calculation in percentage
    //this part still confuses me, will keep working to understand
    if (PErr_now > 10) { //before reaching 10cm error gap
      MotorSpd2 = setvalue;
      ST.motor(1, MotorSpd2); //127
      ST.motor(2, MotorSpd2); //127
      Serial.println("Going down to set depth");
    }
    if (PErr_now < 0) { //if passes desired value(conv)
      ST.motor(1, 0); //0
      ST.motor(2, 0); //0
      Serial.println("To far past depth");

    }
    if (PErr_now < 2 && PErr_now >= 0) { //if 1cm error gap, reset Integral
      IntErr = 0;
      PErr_last = 0;
      MotorSpd2 = setvalue;
      ST.motor(1, 90);
      ST.motor(2, 90);
      Serial.println("Perfect depth");
    }
    if ((PErr_now >= 2) && (PErr_now <= 10)) { //error gap inbtw 1cm and 20cm
      Serial.println("Running PID");
      DErr = PErr_now - PErr_last;
      IntErr += PErr_now;

      M = (kp * PErr_now + ki * IntErr + kd * DErr);

      if (M > 100)
        M = 100;
      if (M < 0) //clip M to 0 if M=-ve
        M = 0;
      PErr_last = PErr_now;
      if (M > 0) {
        if (MotorSpd2 > 0){
        MotorSpd2 = int(MotorSpd2 * (1.0 - M / 100.0));
        ST.motor(1, MotorSpd2);
        ST.motor(2, MotorSpd2);
      }
      else if(MotorSpd2 == 0){
        MotorSpd2 = 25;
        ST.motor(1, MotorSpd2);
        ST.motor(2, MotorSpd2);
      }
      }
      else if (M < 0) {
        // MotorSpd1=int(MotorSpd1*(1.0-M/100.0));wrong
        ST.motor(1, 0); //0
        ST.motor(2, 0); //0
      }
    }
  }
  if (D == "76") {
    ST.motor(1, 0);
    ST.motor(2, 0);
  }
}
