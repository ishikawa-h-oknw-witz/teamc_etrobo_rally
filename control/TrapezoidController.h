#ifndef TRAPEZOID_CONTROLLER_H
#define TRAPEZOID_CONTROLLER_H

class TrapezoidController { 
public:
    TrapezoidController(int startSpeed, int maxSpeed, int rampDistance);

    int calculate(int currentDistance, int targetDistance);

    void setMaxSpeed(int maxSpeed);

private:
    int mStartSpeed;
    int mMaxSpeed;
    int mRampDistance;
};

#endif