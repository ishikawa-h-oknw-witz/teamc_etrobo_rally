#ifndef SCENARIO_RUNNER_H
#define SCENARIO_RUNNER_H

#include "DistanceCalculator.h"
#include "PIDController.h"
#include "Motor.h"
#include "IMU.h"
#include "Logger.h"
#include "kernel.h"
#include "Clock.h"
#include "TrapezoidController.h"

#include <stdlib.h>

using namespace spikeapi;

class ScenarioRunner
{
public:
    ScenarioRunner(
            Motor& leftMotor,
            Motor& rightMotor,
            DistanceCalculator& distanceCalculator,
            PIDController& pidController);

    //基準速度の設定
    void set_speed(int basespeed);

    //前後退走行
    void move(bool direction, int distance);

    //旋回
    void turn(float targetHeading);

private:
    Motor& mLeftMotor;
    Motor& mRightMotor;
    DistanceCalculator& mDistanceCalculator;
    PIDController& mPIDController;
    IMU mImu;
    Clock mClock;
    int mBaseSpeed;
};
#endif