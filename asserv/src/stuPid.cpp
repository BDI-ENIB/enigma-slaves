#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "stuPid.hpp"

PID::PID(double * iinput/*,double * isetpoint*/,double * ioutput,double ikp,double iki,double ikd){
  kp=ikp;
  ki=iki;
  kd=ikd;
  input=iinput;
  //setpoint=isetpoint;
  output=ioutput;
  prevT=-1;
  prevE=0;
  integ=0;
}
void PID::compute(){
  double E=/**setpoint-*/-*input;
  long T=micros();
  double dt=T-prevT;
  if(prevT==-1) dt=0;
  integ+=E*dt;
  double deriv;
  if(dt==0)deriv=0;
  else deriv=(E-prevE)/dt;
  prevT=T;
  prevE=E;
  *output=kp*E+ki*integ+kd*deriv;

  /*if(abs(*output) < STUPID_MAX_ZERO_POWER){
      if(abs(*output) >= STUPID_MIN_ZERO_POWER){
      if(*output>0){
        *output=STUPID_MAX_ZERO_POWER;
      }else{
        *output=-STUPID_MAX_ZERO_POWER;
      }
    }
  }*/
}
void PID::set(double ikp,double iki,double ikd){
  kp=ikp;
  ki=iki;
  kd=ikd;
}
double PID::getP(){
  return kp;
}
double PID::getI(){
  return ki;
}
double PID::getD(){
  return kd;
}
void PID::reset(){
  prevT=-1;
  prevE=0;
  integ=0;
}
