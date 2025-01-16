#include <AccelStepper.h>
#include <math.h>

// ピン設定
const int pulsePin1 = 22;   // モーター1のパルスピン
const int dirPin1   = 23;   // モーター1の方向ピン
const int pulsePin2 = 5;    // モーター2のパルスピン
const int dirPin2   = 18;   // モーター2の方向ピン
const int pulsePin3 = 16;   // モーター3のパルスピン
const int dirPin3   = 19;   // モーター3の方向ピン
const int pulsePin4 = 27;   // モーター4のパルスピン
const int dirPin4   = 25;   // モーター4の方向ピン
const int pulsePin5 = 15;   // モーター5のパルスピン
const int dirPin5   = 4;    // モーター5の方向ピン

AccelStepper stepper1(AccelStepper::DRIVER, pulsePin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, pulsePin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, pulsePin3, dirPin3);
AccelStepper stepper4(AccelStepper::DRIVER, pulsePin4, dirPin4);
AccelStepper stepper5(AccelStepper::DRIVER, pulsePin5, dirPin5);

// モーターの設定
const float microsteps1 = 12800.0f;    // モーター1のマイクロステップ数
const float gearRatio1 = 4.4f;         // モーター1のギア比
float stepsPerRevolution1;

const float microsteps2 = 12800.0f;    // モーター2のマイクロステップ数
const float gearRatio2 = 4.4f;         // モーター2のギア比
float stepsPerRevolution2;

const float microsteps3 = 1600.0f;     // モーター3のマイクロステップ数
const float gearRatio3 = 13.0f;        // モーター3のギア比
float stepsPerRevolution3;

const float microsteps4 = 1600.0f;     // モーター4のマイクロステップ数
const float gearRatio4 = 13.0f;        // モーター4のギア比
float stepsPerRevolution4;

const float microsteps5 = 1600.0f;     // モーター5のマイクロステップ数
const float gearRatio5 = 2.0f;         // モーター5のギア比
float stepsPerRevolution5;

// 各モーターの角度制限
const float minAngle1 = -180.0f;
const float maxAngle1 =  180.0f;

const float minAngle2 = -42.0f;
const float maxAngle2 =  42.0f;

const float minAngle3 = -120.0f;
const float maxAngle3 =  120.0f;

const float minAngle4 = -90.0f;
const float maxAngle4 =  90.0f;

const float minAngle5 =  0.0f;
const float maxAngle5 =  360.0f;

// プログラム開始時の角度
const float startAngle1 =  0.0f;
const float startAngle2 = -42.0f;
const float startAngle3 =  0.0f;
const float startAngle4 =  0.0f;
const float startAngle5 =  0.0f;

// ターゲットステップ数（目標位置）
float targetSteps1 = 0.0f;
float targetSteps2 = 0.0f;
float targetSteps3 = 0.0f;
float targetSteps4 = 0.0f;
float targetSteps5 = 0.0f;

// 速度・加速度設定
const float maxSpeed  = 10000.0f;  // 最大速度
const float maxAccel  = 5000.0f;   // 最大加速度
const float minSpeed  = 100.0f;    // 最小速度
const float minAccel  = 50.0f;     // 最小加速度

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // 各モーターのstepsPerRevolutionを計算
  stepsPerRevolution1 = microsteps1 * gearRatio1;
  stepsPerRevolution2 = microsteps2 * gearRatio2;
  stepsPerRevolution3 = microsteps3 * gearRatio3;
  stepsPerRevolution4 = microsteps4 * gearRatio4;
  stepsPerRevolution5 = microsteps5 * gearRatio5;

  // 5つのステッピングモーターに対し、初期速度・加速度を設定
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(maxAccel);

  stepper2.setMaxSpeed(maxSpeed);
  stepper2.setAcceleration(maxAccel);

  stepper3.setMaxSpeed(maxSpeed);
  stepper3.setAcceleration(maxAccel);

  stepper4.setMaxSpeed(maxSpeed);
  stepper4.setAcceleration(maxAccel);

  stepper5.setMaxSpeed(maxSpeed);
  stepper5.setAcceleration(maxAccel);

  // モーター5を-360度回転させる
  long stepsToMove = -stepsPerRevolution5;
  stepper5.moveTo(stepsToMove);

  // モーター5が移動を完了するまで待機
  while (stepper5.distanceToGo() != 0) {
    stepper5.run();
  }

  // 開始時の位置を設定
  stepper1.setCurrentPosition((startAngle1 / 360.0f) * stepsPerRevolution1);
  stepper2.setCurrentPosition((startAngle2 / 360.0f) * stepsPerRevolution2);
  stepper3.setCurrentPosition((startAngle3 / 360.0f) * stepsPerRevolution3);
  stepper4.setCurrentPosition((startAngle4 / 360.0f) * stepsPerRevolution4);
  stepper5.setCurrentPosition((startAngle5 / 360.0f) * stepsPerRevolution5);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("ANGLE:")) {
      // "ANGLE:" 以降を取り出して","区切りで5つの角度をパースする
      String anglesStr = input.substring(6);
      float angles[5];
      
      // 角度を取り出し
      for (int i = 0; i < 5; i++) {
        int commaIndex = anglesStr.indexOf(',');
        if (commaIndex == -1 && i < 4) {
          // 不足する角度があればデフォルト値（0）を設定
          angles[i] = 0.0f;
          break;
        }
        if (commaIndex == -1) {
          // 最後の角度
          angles[i] = anglesStr.toFloat();
        } else {
          // カンマまでの部分をfloat変換
          angles[i] = anglesStr.substring(0, commaIndex).toFloat();
          anglesStr = anglesStr.substring(commaIndex + 1);
        }
      }

      // 前回のターゲットステップを保存
      float previousTargetSteps1 = targetSteps1;
      float previousTargetSteps2 = targetSteps2;
      float previousTargetSteps3 = targetSteps3;
      float previousTargetSteps4 = targetSteps4;
      float previousTargetSteps5 = targetSteps5;

      // 角度制限 & ターゲットステップ計算
      float angle1 = constrain(angles[0], minAngle1, maxAngle1);
      float angle2 = constrain(angles[1], minAngle2, maxAngle2);
      float angle3 = constrain(angles[2], minAngle3, maxAngle3);
      float angle4 = constrain(angles[3], minAngle4, maxAngle4);
      float angle5 = constrain(angles[4], minAngle5, maxAngle5);

      targetSteps1 = (angle1 / 360.0f) * stepsPerRevolution1;
      targetSteps2 = (angle2 / 360.0f) * stepsPerRevolution2;
      targetSteps3 = (angle3 / 360.0f) * stepsPerRevolution3;
      targetSteps4 = (angle4 / 360.0f) * stepsPerRevolution4;
      targetSteps5 = (angle5 / 360.0f) * stepsPerRevolution5;

      stepper1.moveTo(targetSteps1);
      stepper2.moveTo(targetSteps2);
      stepper3.moveTo(targetSteps3);
      stepper4.moveTo(targetSteps4);
      stepper5.moveTo(targetSteps5);

      
      // 目標位置が1つでも変化した場合のみ、速度と加速度を更新
      bool changed = (previousTargetSteps1 != targetSteps1) ||
                     (previousTargetSteps2 != targetSteps2) ||
                     (previousTargetSteps3 != targetSteps3) ||
                     (previousTargetSteps4 != targetSteps4) ||
                     (previousTargetSteps5 != targetSteps5);

      if (changed) {
        // 各モーターの移動距離を計算
        float distance1 = abs(targetSteps1 - stepper1.currentPosition());
        float distance2 = abs(targetSteps2 - stepper2.currentPosition());
        float distance3 = abs(targetSteps3 - stepper3.currentPosition());
        float distance4 = abs(targetSteps4 - stepper4.currentPosition());
        float distance5 = abs(targetSteps5 - stepper5.currentPosition());

        if ((distance1 == 0.0f) && (distance2 == 0.0f) &&
            (distance3 == 0.0f) && (distance4 == 0.0f) &&
            (distance5 == 0.0f)) {
          stepper1.setMaxSpeed(maxSpeed);
          stepper1.setAcceleration(maxAccel);
          stepper2.setMaxSpeed(maxSpeed);
          stepper2.setAcceleration(maxAccel);
          stepper3.setMaxSpeed(maxSpeed);
          stepper3.setAcceleration(maxAccel);
          stepper4.setMaxSpeed(maxSpeed);
          stepper4.setAcceleration(maxAccel);
          stepper5.setMaxSpeed(maxSpeed);
          stepper5.setAcceleration(maxAccel);
        }
        else {
          // 全てのモーターが異なる移動距離の場合、距離が最大のモーターを基準に各モーターの速度・加速度を調整する
          float distances[5] = { distance1, distance2, distance3, distance4, distance5 };
          float maxDist = 0.0f;
          for (int i = 0; i < 5; i++) {
            if (distances[i] > maxDist) {
              maxDist = distances[i];
            }
          }

          // ratio = (自モーターの距離 / 最大距離)
          // 極端に小さくならないように fmax() で下限を設定
          auto setMotorSpeedAccel = [&](AccelStepper &stepper, float dist) {
            if (maxDist <= 0.0f) {
              // 念のため、万が一 maxDist が 0 のときは全モーター最大値
              stepper.setMaxSpeed(maxSpeed);
              stepper.setAcceleration(maxAccel);
              return;
            }
            float ratio = dist / maxDist;
            ratio = fmax(ratio, 0.1f);
            stepper.setMaxSpeed(fmax(maxSpeed * ratio, minSpeed));
            stepper.setAcceleration(fmax(maxAccel * ratio, minAccel));
          };

          setMotorSpeedAccel(stepper1, distance1);
          setMotorSpeedAccel(stepper2, distance2);
          setMotorSpeedAccel(stepper3, distance3);
          setMotorSpeedAccel(stepper4, distance4);
          setMotorSpeedAccel(stepper5, distance5);
        }
      }
    }
  }

  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  stepper5.run();
}
