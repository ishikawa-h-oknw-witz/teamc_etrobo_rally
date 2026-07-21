#include "ScenarioRunner.h"

// コンストラクタ
ScenarioRunner::ScenarioRunner(
    Motor& leftMotor,
    Motor& rightMotor,
    DistanceCalculator& distanceCalculator,
    PIDController& pidController,
    TrapezoidController& trapezoidController)
    : mLeftMotor(leftMotor),
      mRightMotor(rightMotor),
      mDistanceCalculator(distanceCalculator),
      mPIDController(pidController),
      mTrapezoidController(trapezoidController),
      mBaseSpeed(60)
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
    mImu.resetHeading();                //ジャイロリセット
    mDistanceCalculator.reset();        //距離リセット 
    mPIDController.reset();             //微分項リセット  

    // 直進用PID
    mPIDController.setGain(
        3.0,
        0.0,
        0.0);

    while(mDistanceCalculator.getDistance() < distance){
        float current =
            mDistanceCalculator.getDistance();

        int speed =
            mTrapezoidController.getSpeed(
                current,
                distance
            );

        // 後退の場合は速度を負にする
        if(!direction)
        {
            speed = -speed;
        }

        float heading = mImu.getHeading();
        
        // 目標角度は0°
        float error = 0.0f - heading;
        float correction = mPIDController.calculate(error);

        int leftPower = 0;
        int rightPower = 0;

        leftPower  = speed + correction;
        rightPower = speed - correction;

        // PWM制限
        if (leftPower > 100) leftPower = 100;
        if (leftPower < -100) leftPower = -100;

        if (rightPower > 100) rightPower = 100;
        if (rightPower < -100) rightPower = -100;

        mLeftMotor.setPower(leftPower);
        mRightMotor.setPower(rightPower);

        tslp_tsk(10*1000);   // 約10ms周期
    }

    mLeftMotor.brake();
    mRightMotor.brake();
}

void ScenarioRunner::turn(float targetHeading)
{
    mImu.resetHeading();
    mPIDController.reset();

    mPIDController.setGain(
        0.6,
        0.0,
        0.0);

    while (true)
    {
        float currentHeading = mImu.getHeading();

        float error = targetHeading - currentHeading;

        if (abs(error) < 0.5f) {    // ±1°以内になったら終了
            break;
        }

        // パルス制御
        // 誤差が5°以下になったら最低出力で5msだけ旋回し、一旦停止して再度誤差を確認する。
        // これを繰り返すことで、目標角度付近でのオーバーシュートを抑える。
        if (abs(error) <= 5)
        {
            if (error > 0)
            {
                mLeftMotor.setPower(30);
                mRightMotor.setPower(-30);
            }
            else
            {
                mLeftMotor.setPower(-30);
                mRightMotor.setPower(30);
            }

            tslp_tsk(10 * 1000);

            mLeftMotor.stop();
            mRightMotor.stop();
            tslp_tsk(10 * 1000);

            continue;
        }

        // PID制御
        // 誤差が5°を超える間はPID制御により旋回出力を計算する。
        int turnPower = abs(mPIDController.calculate(error)); 

        //PID計算結果が40以上なら40に制限し、30以下なら30に引き上げる
        //上限を決めるのは安定させるため、下限を決めるのは走行体のスタックを防ぐため
        if (turnPower > 40)
        {
            turnPower = 40;
        }

        if (turnPower < 30) //最低動作出力
        {
            turnPower = 30;
        }

        if (error > 0) {
            mLeftMotor.setPower(turnPower);
            mRightMotor.setPower(-turnPower);
        }
        else {
            mLeftMotor.setPower(-turnPower);
            mRightMotor.setPower(turnPower);
        }
        tslp_tsk(10*1000);   // 約10ms周期
    }
    mLeftMotor.brake();
    mRightMotor.brake();
}