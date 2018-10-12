#ifndef DIFFERENTIAL_CONTROLLER_H
#define DIFFERENTIAL_CONTROLLER_H
#include "stuPid.hpp"
#include "cuPid.hpp"
#include "DifferentialController.hpp"

#include <cmath>

#define MAX_PWM 255

class DifferentialController{
  public:
    /** Constructeur, initialise l'asservissement
     * @dP asserv de distance : proportionnel
     * @dI asserv de distance : intégral
     * @dD asserv de distance : dérivation
     * @aP asserv d'angle : proportionnel
     * @aI asserv d'angle : intégral
     * @aD asserv d'angle : dérivaton
     */
    DifferentialController(double dP,double dI,double dD,double aP,double aI,double aD);
    void update(double currentX,double currentY, double currentAngle);
    void setTarget(double targetAngle, double targetX, double targetY, bool forceForward = true);
    int getLeftMotorCommand();
    int getRightMotorCommand();
    void reconfigPID(double dP,double dI,double dD,double aP,double aI,double aD);
    bool isObjectiveReached();
    void reset();
    double a_Current, a_Target, a_Command;
    double reduc;
    double computedAngle, computedDistance;
  private:
    PID *distancePID;
    CuPID *anglePID;
    double d_Current, d_Target, d_Command;
    double targetX, targetY, targetAngle;
    bool hasReachedObjective;

};
#endif
