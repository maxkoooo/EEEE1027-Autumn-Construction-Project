#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define IN1 13
#define IN2 12
#define IN3 2
#define IN4 A3
#define ENA 11
#define ENB 3

#define RIR A1  //right IR sensor input
#define LIR A2  //left IR sensor input
//#define ROT A3  //rotary encoder input


void send_to_driver(unsigned char bit, int PWML, int PWMR);
void forward(int t, int PWM);
void backward(int t, int PWM);
void stop(int t, int PWM);
void turn(int t, int PWML, int PWMR);
void followLine(int IRL, int IRR, int LT, int RT, int t, int v);