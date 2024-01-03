/*
 *      L298N Motor Driver Library
 *      
 *      Made for Line Following Vehicle
 * 
 *      November 2023
 */

#include <Arduino.h>
#include <car.h>

/*
 *   Hex to Binary Cheat Sheet
 *  ==========================================
 *   A = 1010 -> both motors forward
 *   9 = 1001 -> right forward, left backward
 *   6 = 0110 -> left forward, right backward
 *   5 = 0101 -> both motors backward
 */

/* accepts input as a user defined literal hex operator and converts to 4 bit binary, each bit for each of the 4 motor driver pins  */
void send_to_driver(unsigned char bit, int PWML, int PWMR){
    digitalWrite(IN1, ((bit >> 3) & 0x1)); //right forward
    digitalWrite(IN2, ((bit >> 2) & 0x1)); //right backward
    digitalWrite(IN3, ((bit >> 1) & 0x1)); //left forward
    digitalWrite(IN4, ((bit >> 0) & 0x1)); //left backward

    analogWrite(ENA, PWMR); //right pwm
    analogWrite(ENB, PWML); //left pwm
}

/* forward for time t in milliseconds */
void forward(int t, int PWM){
    send_to_driver(0xA, PWM, PWM);
    delay(t);
}

/* backward for time t in milliseconds */
void backward(int t, int PWM){
    send_to_driver(0x5, PWM, PWM);
    delay(t);
}

/* stop for time t in milliseconds */
void stop(int t, int PWM){
    send_to_driver(0x0, PWM, PWM);
    delay(t);
}

/* turns the vehicle for time t in milliseconds */
void turn(int t, int PWML, int PWMR){
    if(PWML < 0){   //left motor backward, right motor forward
        send_to_driver(0x9, -PWML, PWMR);
        delay(t);
    }
    else if(PWMR < 0){  //right motor backward, left motor forward
        send_to_driver(0x6, PWML, -PWMR);
        delay(t);
    }
    else if((PWML < 0) && (PWMR < 0)){  //both motor backward
        send_to_driver(0x5, -PWML, -PWMR);
        delay(t);
    }
    else{   //both motor forward
        send_to_driver(0xA, PWML, PWMR);
        delay(t);
    }
}

/*  
 *  line following algorithm for 2 IR sensors 
 *
 *  IRL = left IR sensor
 *  IRR = right IR sensor
 *  LT = left IR threshold
 *  RT = right IR threshold
 *  t = time between each subsequent reading
 *  v = line following speed (PWM)
 */
void followLine(int IRL, int IRR, int LT, int RT, int t, int v){
    if((IRL < LT) && (IRR > RT)){
        turn(t, -255, 255);
    }
    else if((IRL > LT) && (IRR < RT)){
        turn(t, 255, -255);
    }
    else if((IRL < LT) && (IRR < RT)){
        stop(t, 0);
    }
    else{
        forward(t, v);
    }
}