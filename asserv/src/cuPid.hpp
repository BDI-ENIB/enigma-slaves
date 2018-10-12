/* simple timestamp useful Pid */
#ifndef CUPID_H
#define CUPID_H

// 2
#define CUPID_MIN_ZERO_POWER 2
#define CUPID_MAX_ZERO_POWER 120
// 2-120
class CuPID
{
  public:
    CuPID(double * iinput, double * isetpoint, double * ioutput, double ikp,double iki,double ikd);
    void compute();
    void set(double kp,double ki,double kd);
    double getP();
    double getI();
    double getD();
    void reset();
    bool isFacingFront();
    void printDebug();
  private:
    double *input,*setpoint,*output;
    double kp,ki,kd,integ,prevE;
    long prevT;
    bool debug = false;
};
#endif
