#include "PIDController.h"
 
// コンストラクタ
PIDController::PIDController()
    : mKp(0.6),
      mKi(0.0),
      mKd(0.1),
      mIntegral(0),
      mPreviousError(0)
{
}

void PIDController::setGain(float kp, float ki, float kd)
{
    mKp = kp;
    mKi = ki;
    mKd = kd;
}

void PIDController::reset()
{
    mIntegral = 0.0f;
    mPreviousError = 0.0f;
}

float PIDController::calculate(float error)
{
    float dt = 0.01f;
    
    // 積分項更新
    mIntegral += error;
 
    // 微分項計算
    float diff = (error - mPreviousError) / dt;
 
    // PID計算
    float turn =
        mKp * error +
        mKi * mIntegral +
        mKd * diff;
 
    mPreviousError = error;

    return turn;
}