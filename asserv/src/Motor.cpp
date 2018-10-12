#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Motor.hpp"

Motor::Motor(){

}

Motor::Motor(int pwm,int p1,int p2){
  pins[0]=pwm;
  pins[1]=p1;
  pins[2]=p2;
  analogWriteFrequency(pwm, 25000);
  for(int i=0;i<3;i++){
    pinMode(pins[i],OUTPUT);
    digitalWrite(pins[i],LOW);
  }
}
void Motor::ARU(){
  speed=0;
  for(int i=0;i<3;i++){
    digitalWrite(pins[i],LOW);
  }
  disabled=true;
}
void Motor::setSpeed(int spd){
  if(!disabled){
    speed=spd;
    if(speed>0){
      digitalWrite(pins[2], LOW);
      digitalWrite(pins[1], HIGH);
    }
    else{
      digitalWrite(pins[1], LOW);
      digitalWrite(pins[2], HIGH);
    }

    analogWrite(pins[0],abs(speed));
  }
}
int Motor::getSpeed(){
  return speed;
}
