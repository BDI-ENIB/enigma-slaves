#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "DifferentialController.hpp"

/*public:
     ** Constructeur, initialise l'asservissement
     * @dP asserv de distance : proportionnel
     * @dI asserv de distance : intégral
     * @dD asserv de distance : intégral
     * @aP asserv d'angle : intégral
     * @aI asserv d'angle : intégral
     * @aD asserv d'angle : intégral
     *
    DifferentialController(double dP,double dI,double dD,double aP,double aI,double aD);
    void update(double currentX,double currentY, double currentAngle);
    void setTarget(double targetAngle, double targetX = NAN, double targetY = NAN, forceForward = false);
    int getLeftMotorCommand();
    int getRightMotorCommand();
    void reconfigPID(double dP,double dI,double dD,double aP,double aI,double aD);
    bool hasReachedObjective();
    void reset();
  private:
    PID *distancePID;
    CuPID *anglePID;
    double d_Current, d_Command;
    double a_Current, a_Target, a_Command;
    double targetX, targetY, targetAngle;
    bool hasReachedObjective;
*/

DifferentialController::DifferentialController(double dP,double dI,double dD,double aP,double aI,double aD){
  targetX = 0;
  targetY = 0;
  targetAngle = 0;

  d_Target = 0;
  a_Target = 0;

  hasReachedObjective = false; // Le robot chercheras à se mettre face à son objectif

  distancePID = new PID(&d_Current/*,&d_Target*/,&d_Command,dP,dI,dD); // Command est un output, l'objectif est tjrs 0
  anglePID = new CuPID(&a_Current,&a_Target,&a_Command,aP,aI,aD); // Command est un output, a_Target varie.
}

void DifferentialController::update(double currentX,double currentY, double currentA){
  // Pre-calcul:

  // Calcul de la distance à l'objectif
  computedDistance=sqrt(pow(targetX-currentX,2)+pow(targetY-currentY,2));

  // Calcul de l'angle à l'objectif
  computedAngle=atan2(targetY-currentY,targetX-currentX);

  while(computedAngle<-PI)computedAngle+=PI;
  while(computedAngle>PI)computedAngle-=PI;

  /* Un modulo pour l'angle (-PI/2, PI/2). ça veux dire qu'il peut reculer pour aller à son objectif
  // TODO: implement forceForward check
  if(abs(abs(currentA)-abs(computedAngle))>PI/2){
    if(computedAngle>0){
      computedAngle-=PI;
    }else{
      computedAngle+=PI;
    }
  }*/

  // Switch de comportement à l'arrivée sur l'objectif
  if(computedDistance<10 || (hasReachedObjective)){
    // reset des PIDs
    if(!hasReachedObjective){
      distancePID->reset();
      anglePID->reset();
      hasReachedObjective = true;
    }

    // commande pid distance:
    d_Target = 0;
    d_Current=computedDistance;
    distancePID->compute();

    //command pid angle:
    a_Current=currentA;
    a_Target=targetAngle;
    anglePID->compute();
  }else{

    //command pid angle:
    a_Current=currentA;
    a_Target=computedAngle;
    anglePID->compute();

    // commande pid distance:
    d_Target = 0;
    d_Current=computedDistance;
    distancePID->compute();
  }

  //On s'assure que les commandes sont envoyées de manière homogènes
  if(abs(d_Command)+abs(a_Command)>MAX_PWM){ // Pour que les commandes soient mitigées de manière homogène
    reduc = max(0.7,MAX_PWM/(abs(d_Command)+abs(a_Command)));
  }else{
    reduc = 1;
  }
}

void DifferentialController::setTarget(double iTargetA, double iTargetX, double iTargetY, bool forceForward){
  this->reset();
  hasReachedObjective = false;
  targetX=iTargetX;
  targetY=iTargetY;
  targetAngle=iTargetA;
}

int out;
int DifferentialController::getLeftMotorCommand(){
  /*if(hasReachedObjective){
    return 0;
  }*/
  out=-a_Command;

  if(abs(computedAngle-a_Current)<3.1415926535/2){
    out-=d_Command;
  }else{
    out+=d_Command;
  }
  return max(-MAX_PWM, min(out*reduc, MAX_PWM));
}

//delay = 0;
int DifferentialController::getRightMotorCommand(){
  /*if(hasReachedObjective){
    return 0;
  }*/
  out=+a_Command;
  if(abs(computedAngle-a_Current)<3.1415926535/2){
    out-=d_Command;
  }else{
    out+=d_Command;
  }
  return max(-MAX_PWM, min(out*reduc, MAX_PWM));
}

void DifferentialController::reset(){
  distancePID->reset();
  anglePID->reset();
}

bool DifferentialController::isObjectiveReached(){
  return hasReachedObjective;
}
