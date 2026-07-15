#include "ScenarioRunner.h"

// コンストラクタ
ScenarioRunner::ScenarioRunner(
    Motor& leftMotor,
    Motor& rightMotor,
    DistanceCalculator& distanceCalculator,
    PIDController& pidController)
    : mLeftMotor(leftMotor),
      mRightMotor(rightMotor),
      mDistanceCalculator(distanceCalculator),
      mPIDController(pidController),
      mBaseSpeed(40)
{
}

void ScenarioRunner::set_speed(int basespeed)
{
    if(basespeed > 100)
    {
        mBaseSpeed = 100;
    }
    else if(basespeed < 0)
    {
        mBaseSpeed = 0;
    }
    else{
        mBaseSpeed = basespeed;
    }
}

void ScenarioRunner::move(bool direction, int distance)
{
    mImu.resetHeading();
    mDistanceCalculator.reset();
    float targetHeading = mImu.getHeading();

    mPIDController.setGain(
        5.0,
        0.0,
        0.2);

    while(mDistanceCalculator.getDistance() < distance){
        float currentHeading = mImu.getHeading();
        int error = targetHeading - currentHeading;
        int correction = mPIDController.calculate(error);

        if(direction == true){
                mLeftMotor.setPower(mBaseSpeed + correction);
                mRightMotor.setPower(mBaseSpeed - correction);
        }
        else{
            mLeftMotor.setPower(-mBaseSpeed - correction);
            mRightMotor.setPower(-mBaseSpeed + correction);
        }
    }
    mLeftMotor.stop();
    mRightMotor.stop();
}

void ScenarioRunner::turn(float targetHeading)
{
    mImu.resetHeading();

    mPIDController.setGain(
        0.6,
        0.0,
        0.2);

    while (true)
    {
        float currentHeading = mImu.getHeading();

        Logger::printf("currentHeading = %.2f\n", currentHeading);

        float error = targetHeading - currentHeading;

        Logger::printf("error = %.2f\n", error);

        if (abs(error) < 0.5f) {    // ±1°以内になったら終了
            break;
        }

        int turnPower = abs(mPIDController.calculate(error));  // 比例制御

        if (turnPower > 40) {
            turnPower = 40;  // 最大出力制限
        }

        if (turnPower < 25) {
            turnPower = 25;   // 動き出せる最低出力
        }

        if (error > 0) {
            mLeftMotor.setPower(turnPower);
            mRightMotor.setPower(-turnPower);
        }
        else {
            mLeftMotor.setPower(-turnPower);
            mRightMotor.setPower(turnPower);
        }
    }
    mLeftMotor.stop();
    mRightMotor.stop();
}