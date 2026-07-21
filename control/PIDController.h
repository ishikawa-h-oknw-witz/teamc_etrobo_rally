#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:
    PIDController();
    
    void setGain(float kp, float ki, float kd);
    
    float calculate(float error);

    void reset();
 
private:
    float mKp;
    float mKi;
    float mKd;
 
    float mIntegral;
 
    float mPreviousError;
};
 
#endif