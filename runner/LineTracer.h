#ifndef LINE_TRACER_H
#define LINE_TRACER_H

#include "ColorSensor.h"
#include "PIDController.h"
#include "Motor.h"
#include "kernel.h"

using namespace spikeapi;

class LineTracer
{
public:
    LineTracer(
        Motor& leftMotor,
        Motor& rightMotor,
        ColorSensor& colorSensor,
        PIDController& pidController);

    void setBaseSpeed(int speed);

    void run();

private:
    Motor& mLeftMotor;

    Motor& mRightMotor;

    ColorSensor& mColorSensor;

    PIDController& mPIDController;

    int mTargetReflection;

    int mBaseSpeed;
};

#endif