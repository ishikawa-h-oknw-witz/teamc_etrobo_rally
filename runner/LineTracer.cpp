#include "LineTracer.h"

// コンストラクタ
LineTracer::LineTracer(
    Motor& leftMotor,
    Motor& rightMotor,
    ColorSensor& colorSensor,
    PIDController& pidController)
    : mLeftMotor(leftMotor),
      mRightMotor(rightMotor),
      mColorSensor(colorSensor),
      mPIDController(pidController),
      mTargetReflection(50),
      mBaseSpeed(60)
{
}

void LineTracer::setBaseSpeed(int speed)
{
    mBaseSpeed = speed;
}

void LineTracer::run()
{
    int turn = 0;

    // 反射光取得
    int reflection =
        mColorSensor.getReflection();

    // 偏差計算
    int error =
        mTargetReflection - reflection;

    // PID制御依頼
    turn = mPIDController.calculate(error);

    // モータ出力
    mLeftMotor.setPower(
        mBaseSpeed - turn);
 
    mRightMotor.setPower(
        mBaseSpeed + turn);

    tslp_tsk(10*1000);   // 約10ms周期
}