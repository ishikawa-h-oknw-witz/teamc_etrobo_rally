#include "app.h"
#include "kernel.h"
#include "LineTracer.h"
#include "PIDController.h"
#include "DistanceCalculator.h"
#include "ScenarioRunner.h"
#include "Motor.h" 
#include "ForceSensor.h" 
#include "ColorSensor.h"
#include "TrapezoidController.h"
#include "Logger.h"
#include "Battery.h"
#include "kernel_cfg.h"

#include <stdio.h>
#include <string.h>

#include "Clock.h"

using namespace spikeapi;

void runGate(int gateNo);   //基準点通過番号を引数にゲート通過から帰還までを行う
void runTopRow(int No);     //横ゲート通過処理　ゲート通過番号①～④
void runMiddleRow(int No);  //縦ゲート通過処理　ゲート通過番号⑤～⑨
void runBottomRow(int No);  //横ゲート通過処理　ゲート通過番号⑩～⑬
void runBasePointToQr();    //基準点から1つ目のQRまでの走行を行う
void runQrToBasePoint();    //1つ目のQRから基準点までの走行を行う
void runQrToQr(int cell);   //QRからQRまでの走行を行う、引数で何マス分進むかを指定
void passGate();            //ゲートを通過して、通過前の場所に戻ってくるまでの走行を行う
void middle_passGate();     //縦ゲート用の通過処理

/* インスタンス生成 */
Motor leftWheel(EPort::PORT_B,Motor::EDirection::COUNTERCLOCKWISE,true);
Motor rightWheel(EPort::PORT_A,Motor::EDirection::CLOCKWISE,true);
ForceSensor forceSensor(EPort::PORT_D);
ColorSensor colorSensor(EPort::PORT_E);
Clock clock;

PIDController pidController;
TrapezoidController trapezoidController;

LineTracer lineTracer(leftWheel, rightWheel, colorSensor, pidController);
DistanceCalculator distanceCalculator(leftWheel, rightWheel);
ScenarioRunner scenarioRunner(leftWheel, rightWheel, distanceCalculator, pidController, trapezoidController);
Logger logger(colorSensor, leftWheel, rightWheel);

//{走行距離(mmまで), 走行速度, 比例ゲイン, 積分ゲイン, 微分ゲイン}
struct Section {
    int distance;
    int speed;
    float kp;
    float ki;
    float kd;
};

//{走行距離(mmまで), 走行速度, 比例ゲイン, 積分ゲイン, 微分ゲイン}
Section sections[] = {
    {600, 100, 0.6, 0.0, 0.2}, //区間１　直進　約60cm
    {900, 70, 0.6, 0.0, 0.4},  //区間２　緩やかなカーブ　約30cm
    {1400, 100, 0.6, 0.0, 0.2},   //区間３　直進　約50cm
    {1700, 70, 0.6, 0.0, 0.4},  //区間４　緩やかなカーブ　約30cm
    {2100, 100, 0.5, 0.0, 0.2},  //区間５　直進　約40cm
    {2600, 70, 0.6, 0.0, 0.4}, //区間６　緩やかなカーブ　約30cm
    {4400, 70, 0.5, 0.0, 0.4},  //区間７　蛇行　約180cm
    {5400, 100, 0.6, 0.0, 0.2}   //区間８　直進　約100cm
};

extern "C" {

void logger_task(intptr_t exinf)
{
    logger.output();
    ext_tsk();
}

}

void main_task(intptr_t exinf)
{
    /* Bluetooth初期化＆接続待ち */
    logger.init();
    sta_cyc(LOGGER_TASK_CYC);

    /* 初期化 */
    bool measuring = false;

    while (!forceSensor.isTouched());
    while (true)
    {
        //テスト用
        if (forceSensor.isTouched())
        {
            // ボタンを離すまで待つ（チャタリング防止）
            while (forceSensor.isTouched())
            {
                tslp_tsk(10000);
            }

            if (!measuring)
            {
                measuring = true;
            }
            else
            {
                leftWheel.stop();
                rightWheel.stop();
                break;
            }
        }

        if (measuring){
            /*
            //HSV取得
            ColorSensor::HSV hsv;
            colorSensor.getHSV(hsv);
            
            lineTracer.setBaseSpeed(50);
            pidController.setGain(
                0.6,
                0.0,
                0.2);

            while(true)
            {
                lineTracer.run();
                // 青
                if (hsv.h >= 200 && hsv.h <= 260 &&
                    hsv.s >= 50 &&
                    hsv.v >= 20)
                {
                    Logger::printf("青検知");
                    leftWheel.stop();
                    rightWheel.stop();
                    break;
                }
                // 赤
                else if ((hsv.h >= 0 && hsv.h <= 20) ||
                    (hsv.h >= 340 && hsv.h <= 360))
                {
                    if (hsv.s >= 50 &&
                        hsv.v >= 20)
                    {
                        Logger::printf("赤検知");
                        leftWheel.stop();
                        rightWheel.stop();
                        break;
                    }
                }
                // 黄
                else if (hsv.h >= 40 && hsv.h <= 80 &&
                    hsv.s >= 50 &&
                    hsv.v >= 20)
                {
                    Logger::printf("黄検知");
                    leftWheel.stop();
                    rightWheel.stop();
                    break;
                }
                // 緑
                else if (hsv.h >= 90 && hsv.h <= 160 &&
                    hsv.s >= 50 &&
                    hsv.v >= 20)
                {
                    Logger::printf("緑検知");
                    leftWheel.stop();
                    rightWheel.stop();
                    break;
                }
            }*/

            scenarioRunner.move(true, 1000);
            tslp_tsk(100000);
            scenarioRunner.turn(90);
            tslp_tsk(100000);

            /*for(int no = 8; no <= 13; no++)
            {
                runGate(no);
                while (!forceSensor.isTouched());
                scenarioRunner.move(true, 40);
                tslp_tsk(100000);
                scenarioRunner.turn(90);
                tslp_tsk(100000);
            }*/
   
            break;
        }
    }
    ext_tsk(); 
}

void runGate(int gateNo)
{
    switch(gateNo)
    {
        case 1:
            // ゲート①攻略
            runTopRow(0);
            break;

        case 2:
            // ゲート②攻略
            runTopRow(1);
            break;

        case 3:
            // ゲート③攻略
            runTopRow(2);
            break;

        case 4:
            // ゲート④攻略
            runTopRow(3);
            break;

        case 5:
            // ゲート⑤攻略
            runMiddleRow(0);
            break;

        case 6:
            // ゲート⑥攻略
            runMiddleRow(1);
            break;

        case 7:
            // ゲート⑦攻略
            runMiddleRow(2);
            break;

        case 8:
            // ゲート⑧攻略
            runMiddleRow(3);
            break;

        case 9:
            // ゲート⑨攻略
            runMiddleRow(4);
            break;

        case 10:
            // ゲート⑩攻略
            runBottomRow(0);
            break;

        case 11:
            // ゲート⑪攻略
            runBottomRow(1);
            break;

        case 12:
            // ゲート⑫攻略
            runBottomRow(2);
            break;

        case 13:
            // ゲート⑬攻略
            runBottomRow(3);
            break;

        default:
            // 不正なゲート番号
            break;
    }
}

//横ゲート通過処理　ゲート通過番号①～④
void runTopRow(int No)
{
    runBasePointToQr();
    runQrToQr(No);
    scenarioRunner.turn(-90);
    tslp_tsk(100000);
    passGate();
    scenarioRunner.turn(-90);
    tslp_tsk(100000);
    runQrToQr(No);
    runQrToBasePoint();
}

//縦ゲート通過処理　ゲート通過番号⑤～⑨
void runMiddleRow(int No)
{
    runBasePointToQr();
    runQrToQr(No);
    middle_passGate();
    scenarioRunner.turn(-180);
    tslp_tsk(100000);
    runQrToQr(No);
    runQrToBasePoint();
}

//横ゲート通過処理　ゲート通過番号⑩～⑬
void runBottomRow(int No)
{
    runBasePointToQr();
    runQrToQr(No);
    scenarioRunner.turn(90);
    tslp_tsk(100000);
    passGate();
    scenarioRunner.turn(90);
    tslp_tsk(100000);
    runQrToQr(No);
    runQrToBasePoint();
}

void runBasePointToQr()
{
    scenarioRunner.move(true, 260);
}

void runQrToBasePoint()
{
    scenarioRunner.move(true, 260);
}

void runQrToQr(int cell)
{
    scenarioRunner.move(true, 250*cell);
}

void passGate()
{
    scenarioRunner.move(true, 260);
    tslp_tsk(100*1000);
    scenarioRunner.move(false, 260);
}

void middle_passGate()
{
    scenarioRunner.move(true, 130);
    tslp_tsk(100*1000);
    scenarioRunner.move(false, 130);
}


