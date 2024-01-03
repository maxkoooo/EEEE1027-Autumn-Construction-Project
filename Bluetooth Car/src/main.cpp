#include <Arduino.h>
#include "car.h"

char bt = 'S';

void setup() {
  Serial.begin(9600);
}

void loop() {
  bt = Serial.read();

  if(bt == 'F'){
    forward(0, 150);
  }
  else if(bt == 'B'){
    backward(0, 150);
  }
  else if(bt == 'L'){
    turn(0, -150, 150);
  }
  else if(bt == 'R'){
    turn(0, 150, -150);
  }
  else if(bt == 'S'){
    stop(0, 0);
  }
}
