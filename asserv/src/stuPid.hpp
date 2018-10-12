/* simple timestamp useful Pid */
#ifndef STUPID_H
#define STUPID_H

#define STUPID_MAX_ZERO_POWER 0
#define STUPID_MIN_ZERO_POWER 0
//0.4
class PID
{
  public:
    PID(double * iinput/*,double * isetpoint*/,double * ioutput,double ikp,double iki,double ikd);
    void compute();
    void set(double kp,double ki,double kd);
    double getP();
    double getI();
    double getD();
    void reset();
  private:
    double *input,*output/*,*setpoint*/;
    double kp,ki,kd,integ,prevE;
    long prevT;
};
#endif
