#ifndef TRAPEZOIDCONTROLLER_H
#define TRAPEZOIDCONTROLLER_H

class TrapezoidController
{
public:
    TrapezoidController();

    void setParameter(
        int startSpeed,
        int maxSpeed,
        int endSpeed,
        float accelDistance,
        float decelDistance);

    int getSpeed(float currentDistance, float totalDistance);

private:
    int mStartSpeed;
    int mMaxSpeed;
    int mEndSpeed;

    float mAccelDistance;
    float mDecelDistance;
};

#endif