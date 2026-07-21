#ifndef SCENARIO_RUNNER_H
#define SCENARIO_RUNNER_H

#include "Motor.h"
#include "DistanceCalculator.h"
#include "PIDController.h"
#include "TrapezoidController.h"
#include "IMU.h"
#include "Clock.h"

using namespace spikeapi;

class ScenarioRunner
{
public:
    ScenarioRunner(
            Motor& leftMotor,
            Motor& rightMotor,
            DistanceCalculator& distanceCalculator,
            PIDController& pidController,
            TrapezoidController& trapezoidController);

    //前後退走行
    void move(bool direction, int distance);

    //旋回
    void turn(float targetHeading);

private:
    Motor& mLeftMotor;
    Motor& mRightMotor;
    DistanceCalculator& mDistanceCalculator;
    PIDController& mPIDController;
    TrapezoidController& mTrapezoidController;
    IMU mImu;
    Clock mClock;
    int mBaseSpeed;
};
#endif