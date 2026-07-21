#include "TrapezoidController.h"

TrapezoidController::TrapezoidController()
    : mStartSpeed(30),
      mMaxSpeed(60),
      mEndSpeed(30),
      mAccelDistance(100.0f),
      mDecelDistance(100.0f)
{
}

void TrapezoidController::setParameter(
    int startSpeed,
    int maxSpeed,
    int endSpeed,
    float accelDistance,
    float decelDistance)
{
    mStartSpeed = startSpeed;
    mMaxSpeed = maxSpeed;
    mEndSpeed = endSpeed;
    mAccelDistance = accelDistance;
    mDecelDistance = decelDistance;
}

int TrapezoidController::getSpeed(
    float currentDistance,
    float totalDistance)
{
    float remainingDistance = totalDistance - currentDistance;

    // 加速区間
    if (currentDistance < mAccelDistance)
    {
        return mStartSpeed +
            (mMaxSpeed - mStartSpeed)
            * currentDistance / mAccelDistance;
    }

    // 減速区間
    if (remainingDistance < mDecelDistance)
    {
        return mEndSpeed +
            (mMaxSpeed - mEndSpeed)
            * remainingDistance / mDecelDistance;
    }

    // 定速区間
    return mMaxSpeed;
}