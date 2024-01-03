#include <Arduino.h>
#include <car.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int button;
int PID = 0;
int buzzertoggle = 0;

int initPulseR = digitalRead(ROT);
int initPulseL = digitalRead(LOT);
int counterR = 0;
int counterL = 0;

float targetL = 70;
float targetR = 70;
float PWML;
float PWMR;

float error = 0.0;
float lasterror = 0.0;
float P;
float D;
float Kp = 0.4;
float Kd = 0.2;

float radius = 0.0325;
float distance = 0.0;


void setup() {
    Serial.begin(9600);
    lcd.begin(16,2);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // pinMode(LIR, INPUT);
    // pinMode(RIR, INPUT);
    pinMode(LOT, INPUT);
    pinMode(ROT, INPUT);
}

void loop() {
    button = analogRead(BUT);
    if((button >= 600) && (button <= 800)){
        if(PID == 0){
            PID = 1;
        }
        else if(PID == 1){
            PID = 2;
        }
        else if(PID == 2){
            PID = 0;
        }
        delay(250);
    }

    /* Encoder Reading */
    if(PID == 1){
        int pulseR = digitalRead(ROT);
        if(pulseR != initPulseR){
            counterR++;
        }
        initPulseR = pulseR;

        int pulseL = digitalRead(LOT);
        if(pulseL != initPulseL){
            counterL++;
        }
        initPulseL = pulseL;

        error = counterL - counterR;

        P = error;
        D = error - lasterror;

        lasterror = error;

        PWML = targetL - ((Kp*P) + (Kd*D));
        PWMR = targetR + ((Kp*P) + (Kd*D));

        if(PWML > 90){
            PWML = 90;
        }
        if(PWMR > 90){
            PWMR = 90;
        }
        if(PWML < 0){
            PWML = 0;
        }
        if(PWMR < 0){
            PWMR = 0;
        }

        lcd.clear();
        lcd.print("Left: ");
        lcd.print(PWML);
        lcd.setCursor(0,1);
        lcd.print("Right: ");
        lcd.print(PWMR);

        turn(0, PWML, PWMR);
    }
    else if(PID == 0){
        lcd.clear();
        lcd.print("stop");
        stop(0, 0);
    }
    else{
        lcd.clear();
        lcd.print("NO PID Control");
        turn(0, 75, 65);
    }
}