#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "car.h"


int t = 0;  //time in milliseconds

/* LCD Stuff */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
/*
int initPulse = digitalRead(ROT);
int counter = 0;
float radius = 0.0325;
*/
/* MPU Stuff */
Adafruit_MPU6050 mpu;
int rampswitch = 0;
float theta = 0;
float rotstarttime = 0;
float rotcurrenttime = 0;
float rottime = 0;


void setup() {
  Serial.begin(115200);
  lcd.begin(16,2);

  unsigned long starttime = millis()/1000;

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LIR, INPUT);
  pinMode(RIR, INPUT);
  //pinMode(ROT, INPUT);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(5);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // set accelerometer range to +-8G
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);	// set gyro range to +- 500 deg/s
}

void loop() {
  /* Motion Initialisation */
  int IRL = analogRead(LIR);
  int IRR = analogRead(RIR);

  int LT = 500; //left IR threshold
  int RT = 500; //right IR threshold
  int t = 0;  //period between each reading


  /* Printing Distance */
  /*
  int pulse = digitalRead(ROT);

  if(pulse != initPulse){
    counter++;

    lcd.clear();
    lcd.print("Distance: ");
    lcd.print(counter*0.05*PI*radius);
    lcd.print("m");
  }

  initPulse = pulse;
  */
  
  /* MPU 6050 */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float az = a.acceleration.z;
  float gx = g.gyro.x;


 /* Ramp Sequence */
  if((rampswitch == 0) && ((az >= 2.0) || (az <= -2.0))){ //detects ramp
    /* Print Angle */
    float currentAngle = asin(az/9.81)*180/PI;
    lcd.clear();
    lcd.print("Ramp Angle: ");
    lcd.print(currentAngle);
    lcd.print("deg");

    followLine(IRL, IRR, LT, RT, t, 120);

    rampswitch = 1; //initiates ramp sequence
  }
  else if((rampswitch == 1) && ((az < 2.0) || (az > -2.0))){ //stop for 4 seconds on ramp
    // followLine(IRL, IRR, LT, RT, 500, 120); //ensure vehicle is on the top of the ramp

    for(int i=1;i<5;i++){ //stop for 4 seconds and display time
      lcd.setCursor(0,1);
      lcd.print("stop: ");
      lcd.print(i);
      lcd.print("s");
      stop(1000, 0);  
    }
   
    rotstarttime = millis()/1000;
    
    rampswitch = 2;
  }
  else if((rampswitch == 2)){  //rotates 360 degrees on ramp
    rotcurrenttime = millis()/1000;
    rottime = (rotcurrenttime - rotstarttime);

    if((gx > 0.3) || (gx < -0.3)){
      theta = gx*rottime; //total angle rotated
    }

    lcd.clear();
    lcd.print(rottime);
    lcd.setCursor(0,1);
    lcd.print(theta);
    
    if((theta <= (2*PI)) && (theta >= (-2*PI))){  //turn until total angle reaches 360 degrees
      turn(t, 100, -100);
    }
    else{
      rampswitch = 3;
    }
  }
  else{ //default line following
    followLine(IRL, IRR, LT, RT, t, 60); 
  }
}

/* Code Holding Bay */
/*
*/