#include "TrapezoidController.h"

TrapezoidController::TrapezoidController(
    int startSpeed,
    int maxSpeed,
    int rampDistance)
    : mStartSpeed(startSpeed),
      mMaxSpeed(maxSpeed),
      mRampDistance(rampDistance)
{
}

void TrapezoidController::setMaxSpeed(int maxSpeed)
{
    if(maxSpeed > 100)
    {
        mMaxSpeed = 100;
    }
    else if(maxSpeed < 0)
    {
        mMaxSpeed = 0;
    }
    else{
        mMaxSpeed = maxSpeed;
    }
}

int TrapezoidController::calculate(
    int currentDistance,
    int targetDistance)
{
    int remainDistance = targetDistance - currentDistance;

    // 加速
    if (currentDistance < mRampDistance)
    {
        return mStartSpeed +
            (mMaxSpeed - mStartSpeed)
            * currentDistance / mRampDistance;
    }

    // 減速
    if (remainDistance < mRampDistance)
    {
        return mStartSpeed +
            (mMaxSpeed - mStartSpeed)
            * remainDistance / mRampDistance;
    }

    // 等速
    return mMaxSpeed;
}