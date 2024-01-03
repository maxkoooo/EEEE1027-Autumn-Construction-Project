#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "car.h"


/* LCD Stuff */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int initPulse = digitalRead(ROT);
int counter = 0;
float radius = 0.0325;
float distance = 0;
    
unsigned long starttime;
unsigned long currenttime;
int movetime = 0;


Adafruit_MPU6050 mpu;
float ax;
float az;
float gz;
int rampswitch = 0;
float theta = 0;
float rotstarttime = 0;
float rotcurrenttime = 0;
float rottime = 0;


void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LIR, INPUT);
  pinMode(RIR, INPUT);
  pinMode(ROT, INPUT);

  
  if(!mpu.begin()){
    Serial.println("no");
  }
  else{
    Serial.println("yes");
  }
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setInterruptPinLatch(true);	// Keep it latched. Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // set accelerometer range to +-8G
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);	// set gyro range to +- 500 deg/s
}

void loop() {
  /* Motion Initialisation */
  int IRL = analogRead(LIR);
  int IRR = analogRead(RIR);

  int LT = 500; //left IR threshold
  int RT = 500; //right IR threshold
  int t = 0;  //period between each reading  
  
  
  /* MPU 6050 */
  if(rampswitch == 0){
    forward(1500, 150);
    stop(0,0);
    rampswitch = 1;
  }
  else if(rampswitch == 1){
    delay(2500);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x;

    float currentAngle = asin(ax/9.81)*180/PI;
    Serial.println(currentAngle);

    lcd.clear();
    lcd.print("Ramp Angle: ");
    lcd.setCursor(0,1);
    lcd.print(currentAngle);
    lcd.print("deg");

    delay(3000);

    rampswitch = 2;
  }
  else if(rampswitch == 2){
    turn(1000, 170, 150); // going up the ramp

    for(int i=1;i<5;i++){ //stop for 4 seconds and display time
        lcd.clear();
        lcd.print("stop: ");
        lcd.print(i);
        lcd.print("s");
        stop(1000, 0);  
    }    
    
    turn(1000, -255, 255);  //spin
    stop(1000, 0);

    rampswitch = 3;
  }
  else if(rampswitch == 3){// going down slope
    followLine(IRL, IRR, LT, RT, t, 80);

    if(movetime <= 1000){
      movetime++;
      delay(1);
    }
    else{
      movetime = 0;
      forward(500, 100);
      rampswitch = 4;
    }
  }
  else if(rampswitch == 4){
    followLine(IRL, IRR, LT, RT, t, 60);
    
    if(distance <= 0.8){ //move for 80cm
      int pulse = digitalRead(ROT);

      if(pulse != initPulse){
        counter++;
      }
      initPulse = pulse;

      distance = counter*0.05*PI*radius;
      lcd.clear();
      lcd.print("Distance: ");
      lcd.print(distance);
      lcd.print("m");
    }
    else{
      rampswitch = 5;
    }
  }
  else if(rampswitch == 5){
    for(int i=1;i<4;i++){ //stop for 3 seconds and display time
        lcd.setCursor(0,1);
        lcd.print("stop: ");
        lcd.print(i);
        lcd.print("s");
        stop(1000, 0);  
    }
    
    forward(100, 200);

    starttime = millis();
    rampswitch = 6;
  }
  else{
    /* print distance */
    int pulse = digitalRead(ROT);

    if(pulse != initPulse){
      counter++;
    }
    initPulse = pulse;

    distance = counter*0.05*PI*radius;
    lcd.clear();
    lcd.print("Distance: ");
    lcd.print(distance);
    lcd.print("m");
    
    /* count time */
    currenttime = millis();
    int time = (currenttime - starttime)/1000;
    lcd.setCursor(0,1);
    lcd.print("Time: ");
    lcd.print(time);
    lcd.print("s");
    
    followLine(IRL, IRR, LT, RT, t, 60);
  }
}

/* Code Holding Bay */
/*
    followLine(IRL, IRR, LT, RT, t, 60);

    if(movetime <= 1000){
      movetime++;
      delay(1);
    }
    else{
      movetime = 0;
      forward(1000, 60);
      rampswitch = 4;
    }

  Wire.requestFrom(10, 1); // transmit to device #9
  byte ax = Wire.read();
  
  // Ramp Sequence
  if((rampswitch = 0) && ((ax >= 2.0) || (ax <= -2.0)) && (az <= 9.0)){ //detects ramp
    rampswitch = 1;
  }

  if(rampswitch == 1){
    if(((ax >= 1.0) || (ax <= -1.0)) && (az <= 9.0)){
      followLine(IRL, IRR, LT, RT, t, 100);

      //Print Angle
      float currentAngle = asin(ax/9.81)*180/PI;

      lcd.clear();
      lcd.print("Ramp Angle: ");
      lcd.setCursor(0,1);
      lcd.print(currentAngle);
      lcd.print("deg");
    }
    else{
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
  }
  else if((rampswitch == 2)){  //rotates 360 degrees on ramp
    rotcurrenttime = millis()/1000;
    rottime = (rotcurrenttime - rotstarttime);

    if((gz > 0.1) || (gz < -0.1)){
      theta = gz*rottime; //total angle rotated
    }

    lcd.clear();
    lcd.print(rottime);
    lcd.setCursor(0,1);
    lcd.print(theta);
    
    if((theta <= (2*PI)) && (theta >= (-2*PI))){  //turn until total angle reaches 360 degrees
      turn(t, 120, -120);
    }
    else{
      stop(1000, 0);
      rampswitch = 3;
    }
  }
  else if(rampswitch == 3){
    followLine(IRL, IRR, LT, RT, t, 60); 
  }
  else{ //default line following
    followLine(IRL, IRR, LT, RT, t, 60); 
  }
*/

