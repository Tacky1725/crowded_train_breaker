// 可変抵抗は時計回りを角度が増加する方向となるに接続すること
#include <freertos/queue.h>

const int motorPinA1 = 4;  // アクチュエータA 用 IN1
const int motorPinA2 = 5;  // アクチュエータA 用 IN2
const int motorPinB1 = 15; // アクチュエータB 用 IN1
const int motorPinB2 = 2;  // アクチュエータB 用 IN2

const int potentio_SOCKET_FB = 34; // 可変抵抗器のアナログ入力ソケット番号
const int potentio_SOCKET_LR = 35; // 可変抵抗器のアナログ入力ソケット番号

const int SPEED_FAST = 255;
const int SPEED_SLOW = 255 / 3;
const int SPEED_VERY_SLOW = 255 / 4;

const int TOLERANCE = 0; // 傾き制御の角度許容誤差

int initialAngleFB = 0;
int initialAnalogInputFB = 0;

int initialAngleLR = 0;
int initialAnalogInputLR = 0;

int currentAngleFB = 0;
int currentAnalogInputFB = 0;

int currentAngleLR = 0;
int currentAnalogInputLR = 0;

int stopFlag = 0;

// 緊急停止コマンド関連
const char EMERG1 = 'Q'; // 緊急コマンド（大文字Q）
const char EMERG2 = 'q'; // 予備（小文字q）
volatile bool emergency_requested = false;

extern QueueHandle_t gActuatorCommandQueue;
// 緊急コマンドをポーリング（受信していたら true を返す）
/**
 * @brief Check the actuator queue for emergency stop requests.
 * @return true if emergency has been requested.
 */
bool pollEmergency()
{
  if (gActuatorCommandQueue != nullptr)
  {
    char queued;
    char buffered[16];
    int bufferedCount = 0;

    while (xQueueReceive(gActuatorCommandQueue, &queued, 0) == pdTRUE)
    {
      if (queued == EMERG1 || queued == EMERG2)
      {
        emergency_requested = true;
        Serial.println("\n!!! EMERGENCY RECEIVED !!!");
      }
      else
      {
        if (bufferedCount < 16)
        {
          buffered[bufferedCount++] = queued;
        }
      }
    }

    for (int i = 0; i < bufferedCount; ++i)
    {
      xQueueSend(gActuatorCommandQueue, &buffered[i], 0);
    }

    if (emergency_requested)
    {
      return true;
    }
  }
  return emergency_requested;
}

/**
 * @brief Initialize actuator pins and calibrate initial angles.
 */
void setupActuator()
{
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(potentio_SOCKET_FB, INPUT);
  pinMode(potentio_SOCKET_LR, INPUT);

  Serial.println("Actuator Program started.");

  // 角度の初期値（水平時の値）を取得
  resetinitialAngleFB();
  resetinitialAngleLR();
}

/**
 * @brief Main actuator command loop driven by queued serial inputs.
 */
void runActuatorLoop()
{
  getAngleFB();
  getAngleLR();
  printAngle();

  if (gActuatorCommandQueue == nullptr)
    return;

  char c;
  if (xQueueReceive(gActuatorCommandQueue, &c, 0) != pdTRUE)
    return;

  if (c == '\r' || c == '\n')
    return;

  stopA();
  stopB();
  getAngleFB();
  getAngleLR();
  printAngle();

  Serial.print("Command: ");
  Serial.print(c);

  switch (c)
  {
  case EMERG1:
  case EMERG2:
    emergency_requested = true;
    stopFlag = -1;
    stopA();
    stopB();
    Serial.print("(Emergency stop)");
    break;

  // ?????
  case 'W': // ??
    forwardSlowA();
    reverseSlowB();
    break;

  case 'S': // ???
    reverseSlowA();
    forwardSlowB();
    break;

  case 'A': // ??
    // moveToAngle(20); // ????????0????
    reverseSlowA();
    reverseSlowB();
    break;

  case 'D': // ??
    // moveToAngle(-20); // ?????????0????
    forwardSlowA();
    forwardSlowB();
    break;

  case 'X': // ??????????
    moveToAngleFrontBack(0);
    break;

  case 'Z': // ??????????
    moveToAngleLeftRight(0);
    break;

  // ?????
  case 'U': // ??
    forwardFastA();
    reverseFastB();
    break;

  case 'J': // ???
    reverseFastA();
    forwardFastB();
    break;

  case 'H': // ??
    reverseFastA();
    reverseFastB();
    break;

  case 'K': // ??
    forwardFastA();
    forwardFastB();
    break;

  case 'O': // ?????
    moveFrontBack(1000);
    break;

  case 'P': // ?????
    moveLeftRight(1000);
    break;

  case 'L': // ???????????
    moveLeftRightVerySlow(1000);
    break;

  case 'G': // ????????
    moveGatanGoton(10);
    break;

  case 'R':
    resetinitialAngleFB();
    resetinitialAngleLR();
    break;

  default: // ?????? = ??
    stopA();
    stopB();
    Serial.print("(Unknown key)");
    break;
  }
  Serial.println();
}

/**
 * @brief Calibrate the front/back neutral angle from analog input.
 */
void resetinitialAngleFB()
{
  initialAnalogInputFB = getAnalogInputFB();
  int initialAngleFB1 = getAngleFB();
  delay(1000);
  int initialAngleFB2 = getAngleFB();
  delay(1000);
  int initialAngleFB3 = getAngleFB();
  initialAngleFB = (initialAngleFB1 + initialAngleFB2 + initialAngleFB3) / 3; // 平均値を取る

  Serial.print("initialAngleFB[1-3]: ");
  Serial.print(initialAngleFB1);
  Serial.print(", ");
  Serial.print(initialAngleFB2);
  Serial.print(", ");
  Serial.println(initialAngleFB3);

  Serial.print("initialAnalogInputFB: ");
  Serial.print(initialAnalogInputFB);
  Serial.print(", initialAngleFB: ");
  Serial.println(initialAngleFB);
  Serial.println("Ready to receive commands.");
}

/**
 * @brief Calibrate the left/right neutral angle from analog input.
 */
void resetinitialAngleLR()
{
  initialAnalogInputLR = getAnalogInputLR();
  int initialAngleLR1 = getAngleLR();
  delay(1000);
  int initialAngleLR2 = getAngleLR();
  delay(1000);
  int initialAngleLR3 = getAngleLR();
  initialAngleLR = (initialAngleLR1 + initialAngleLR2 + initialAngleLR3) / 3; // 平均値を取る

  Serial.print("initialAngleLR[1-3]: ");
  Serial.print(initialAngleLR1);
  Serial.print(", ");
  Serial.print(initialAngleLR2);
  Serial.print(", ");
  Serial.println(initialAngleLR3);

  Serial.print("initialAnalogInputLR: ");
  Serial.print(initialAnalogInputLR);
  Serial.print(", initialAngleLR: ");
  Serial.println(initialAngleLR);
  Serial.println("Ready to receive commands.");
}

// 前後に振動
/**
 * @brief Shake front/back direction repeatedly while aborting on emergency.
 * @param times Number of oscillations.
 */
void moveFrontBack(int times)
{
  clearEmergency();

  reverseSlowA();
  forwardSlowB();
  if (delayWithAbort(250))
  {
    stopA();
    stopB();
    moveToAngleFrontBack(0);
    return;
  }
  stopA();
  stopB();

  for (int count = 1; count <= times; count++)
  {
    if (pollEmergency())
    {
      stopA();
      stopB();
      moveToAngleFrontBack(0);
      return;
    }

    forwardSlowA(); // 台時計回り
    reverseSlowB();
    if (delayWithAbort(400))
    {
      stopA();
      stopB();
      moveToAngleFrontBack(0);
      return;
    }
    stopA();
    stopB();

    if (pollEmergency())
    {
      stopA();
      stopB();
      moveToAngleFrontBack(0);
      return;
    }

    reverseSlowA(); // 台時計回り
    forwardSlowB();
    if (delayWithAbort(500))
    {
      stopA();
      stopB();
      moveToAngleFrontBack(0);
      return;
    }
    stopA();
    stopB();
  }

  stopA();
  stopB();

  forwardSlowA();
  reverseSlowB();
  if (delayWithAbort(250))
  {
    stopA();
    stopB();
    moveToAngleFrontBack(0);
    return;
  }
  stopA();
  stopB();
}

// 左右に振動
/**
 * @brief Shake left/right direction repeatedly while aborting on emergency.
 * @param times Number of oscillations.
 */
void moveLeftRight(int times)
{
  clearEmergency();

  if (delayWithAbort(100))
  {
    stopA();
    stopB();
    moveToAngleLeftRight(0);
    return;
  }

  forwardSlowA();
  forwardSlowB();
  if (delayWithAbort(400))
  {
    stopA();
    stopB();
    moveToAngleLeftRight(0);
    return;
  }
  stopA();
  stopB();

  for (int count = 1; count <= times; count++)
  {
    if (pollEmergency())
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      return;
    }

    reverseSlowA();
    reverseSlowB();
    if (delayWithAbort(770))
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      return;
    }
    stopA();
    stopB();

    if (pollEmergency())
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      return;
    }

    forwardSlowA();
    forwardSlowB();
    if (delayWithAbort(800))
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      moveToAngleFrontBack(0);
      return;
    }
    stopA();
    stopB();
  }

  stopA();
  stopB();

  reverseSlowA();
  reverseSlowB();
  if (delayWithAbort(400))
  {
    stopA();
    stopB();
    moveToAngleLeftRight(0);
    moveToAngleFrontBack(0);
    return;
  }
  stopA();
  stopB();
}

// 左右に振動（ゆっくり）
/**
 * @brief Slow oscillation left/right with emergency abort support.
 * @param times Number of oscillations.
 */
void moveLeftRightVerySlow(int times)
{
  clearEmergency();

  if (delayWithAbort(100))
  {
    stopA();
    stopB();
    moveToAngleLeftRight(0);
    return;
  }

  forwardVerySlowA();
  forwardVerySlowB();
  if (delayWithAbort(400))
  {
    stopA();
    stopB();
    moveToAngleLeftRight(0);
    return;
  }
  stopA();
  stopB();

  for (int count = 1; count <= times; count++)
  {
    if (pollEmergency())
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      return;
    }

    reverseVerySlowA();
    reverseVerySlowB();
    if (delayWithAbort(2000))
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      moveToAngleFrontBack(0);
      return;
    }
    stopA();
    stopB();

    if (pollEmergency())
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      return;
    }

    forwardVerySlowA();
    forwardVerySlowB();
    if (delayWithAbort(2100))
    {
      stopA();
      stopB();
      moveToAngleLeftRight(0);
      moveToAngleFrontBack(0);
      return;
    }
    stopA();
    stopB();
  }

  stopA();
  stopB();

  reverseVerySlowA();
  reverseVerySlowB();
  if (delayWithAbort(400))
  {
    stopA();
    stopB();
    moveToAngleLeftRight(0);
    return;
  }
  stopA();
  stopB();
}

/**
 * @brief Perform a stepped "gatan goton" motion pattern.
 * @param times Number of repetitions.
 */
void moveGatanGoton(int times)
{
  stopA();
  stopB();
  for (int count = 1; count <= times; count++)
  {
    forwardSlowA();
    forwardSlowB();
    delay(120);
    stopA();
    stopB();
    reverseSlowA();
    reverseSlowB();
    delay(150);
    stopA();
    stopB();
    delay(1000);
  }
  stopA();
  stopB();
  return;
}

/**
 * @brief Print current and initial angles/inputs for debugging.
 */
void printAngle()
{
  Serial.print("initialInput FB: ");
  Serial.print(initialAnalogInputFB);
  Serial.print(", initialAngle FB: ");
  Serial.print(initialAngleFB);
  Serial.print(", currentInput FB: ");
  Serial.print(currentAnalogInputFB);
  Serial.print(", currentAngle FB: ");
  Serial.print(currentAngleFB);

  Serial.print(",\tinitialInput LR: ");
  Serial.print(initialAnalogInputLR);
  Serial.print(", initialAngle LR: ");
  Serial.print(initialAngleLR);
  Serial.print(", currentInput LR: ");
  Serial.print(currentAnalogInputLR);
  Serial.print(", currentAngle LR: ");
  Serial.println(currentAngleLR);
}

// 角度を指定して前後移動（後ろが正）
/**
 * @brief Drive to target front/back relative angle with tolerance and abort.
 * @param targetRelativeAngle Target angle offset from initial calibration.
 */
void moveToAngleFrontBack(int targetRelativeAngle)
{
  clearEmergency(); // ← 直前の緊急状態をクリア
  Serial.print("move to ");
  Serial.println(targetRelativeAngle);

  int targetAngleFB = initialAngleFB + targetRelativeAngle;

  while (1)
  {
    if (pollEmergency())
    { // ★ 追加：緊急コマンド検知
      stopA();
      stopB();
      Serial.println("ABORTED: moveToAngleFrontBack");
      return;
    }

    Serial.print("initialAngleFB: ");
    Serial.print(initialAngleFB);
    Serial.print("\t targetRelativeAngleFB: ");
    Serial.print(targetAngleFB);
    Serial.print("\t currentAngleFB: ");
    int currentAngle = getAngleFB();
    Serial.println(currentAngle);

    if (TOLERANCE < currentAngle - targetAngleFB)
    {
      forwardSlowA();
      reverseSlowB();
    }
    else if (TOLERANCE < targetAngleFB - currentAngle)
    {
      reverseSlowA();
      forwardSlowB();
    }
    else
    {
      break;
    }
  }
  stopA();
  stopB();
}

// 角度を指定して左右移動（左が正）
/**
 * @brief Drive to target left/right relative angle with tolerance and abort.
 * @param targetRelativeAngle Target angle offset from initial calibration.
 */
void moveToAngleLeftRight(int targetRelativeAngle)
{
  clearEmergency(); // ← 直前の緊急状態をクリア
  Serial.print("move to ");
  Serial.println(targetRelativeAngle);

  int targetAngleLR = initialAngleLR + targetRelativeAngle;

  while (1)
  {
    if (pollEmergency())
    { // ★ 追加：緊急コマンド検知
      stopA();
      stopB();
      Serial.println("ABORTED: moveToAngleLeftRight");
      return;
    }

    Serial.print("initialAngleLR: ");
    Serial.print(initialAngleLR);
    Serial.print("\t targetRelativeAngleLR: ");
    Serial.print(targetAngleLR);
    Serial.print("\t currentAngleLR: ");
    int currentAngle = getAngleLR();
    Serial.println(currentAngle);

    if (TOLERANCE < currentAngle - targetAngleLR)
    {
      forwardSlowA();
      forwardSlowB();
    }
    else if (TOLERANCE < targetAngleLR - currentAngle)
    {
      reverseSlowA();
      reverseSlowB();
    }
    else
    {
      break;
    }
  }
  stopA();
  stopB();
}

/**
 * @brief Demo drive mode that picks random FB targets until stopped.
 */
void driveMode()
{
  while (1)
  {
    if (stopFlag == -1)
    { // 中断コマンドでdriveModeからも抜けられるように
      Serial.println("Command: Q (stop driveMode)");
      break;
    }
    else
    {
      int randomInt = random(-10, 10); // 新しい目標角度を設定
      Serial.print("randomInt: ");
      Serial.println(randomInt);
      moveToAngleFrontBack(randomInt);
    }
  }
  stopA();
  stopB();
}

/**
 * @brief Read and filter FB analog input and convert to angle.
 * @return Current FB angle in degrees.
 */
int getAngleFB()
{
  int analogin1 = getAnalogInputFB();
  delay(5);
  int analogin2 = getAnalogInputFB();
  delay(5);
  int analogin3 = getAnalogInputFB();
  int analogInput = (analogin1 + analogin2 + analogin3) / 3;

  int angle = map(analogInput, 0, 4095, 0, 270);      // アナログ入力レンジ => 角度指示レンジに変換
  int percentage = map(analogInput, 0, 4095, 0, 100); // アナログ入力レンジ => アナログ出力レンジに変換

  currentAnalogInputFB = analogInput;
  currentAngleFB = angle;

  return angle;
}

/**
 * @brief Read and filter LR analog input and convert to angle.
 * @return Current LR angle in degrees.
 */
int getAngleLR()
{
  int analogin1 = getAnalogInputLR();
  delay(5);
  int analogin2 = getAnalogInputLR();
  delay(5);
  int analogin3 = getAnalogInputLR();
  int analogInput = (analogin1 + analogin2 + analogin3) / 3;

  int angle = map(analogInput, 0, 4095, 0, 270);      // アナログ入力レンジ => 角度指示レンジに変換
  int percentage = map(analogInput, 0, 4095, 0, 100); // アナログ入力レンジ => アナログ出力レンジに変換

  currentAnalogInputLR = analogInput;
  currentAngleLR = angle;

  return angle;
}

/**
 * @brief Read raw analog value from FB potentiometer.
 * @return Raw ADC value.
 */
int getAnalogInputFB()
{
  return analogRead(potentio_SOCKET_FB);
}

/**
 * @brief Read raw analog value from LR potentiometer.
 * @return Raw ADC value.
 */
int getAnalogInputLR()
{
  return analogRead(potentio_SOCKET_LR);
}

/** @brief Drive actuator A forward at fast speed. */
void forwardFastA()
{
  forward(motorPinA1, motorPinA2, SPEED_FAST);
}

/** @brief Drive actuator B forward at fast speed. */
void forwardFastB()
{
  forward(motorPinB1, motorPinB2, SPEED_FAST);
}

/** @brief Drive actuator A reverse at fast speed. */
void reverseFastA()
{
  reverse(motorPinA1, motorPinA2, SPEED_FAST);
}

/** @brief Drive actuator B reverse at fast speed. */
void reverseFastB()
{
  reverse(motorPinB1, motorPinB2, SPEED_FAST);
}

/** @brief Drive actuator A forward at slow speed. */
void forwardSlowA()
{
  forward(motorPinA1, motorPinA2, SPEED_SLOW);
}

/** @brief Drive actuator B forward at slow speed. */
void forwardSlowB()
{
  forward(motorPinB1, motorPinB2, SPEED_SLOW);
}

/** @brief Drive actuator A reverse at slow speed. */
void reverseSlowA()
{
  reverse(motorPinA1, motorPinA2, SPEED_SLOW);
}

/** @brief Drive actuator B reverse at slow speed. */
void reverseSlowB()
{
  reverse(motorPinB1, motorPinB2, SPEED_SLOW);
}

/** @brief Drive actuator A forward at very slow speed. */
void forwardVerySlowA()
{
  forward(motorPinA1, motorPinA2, SPEED_VERY_SLOW);
}

/** @brief Drive actuator B forward at very slow speed. */
void forwardVerySlowB()
{
  forward(motorPinB1, motorPinB2, SPEED_VERY_SLOW);
}

/** @brief Drive actuator A reverse at very slow speed. */
void reverseVerySlowA()
{
  reverse(motorPinA1, motorPinA2, SPEED_VERY_SLOW);
}

/** @brief Drive actuator B reverse at very slow speed. */
void reverseVerySlowB()
{
  reverse(motorPinB1, motorPinB2, SPEED_VERY_SLOW);
}

/** @brief Stop actuator A softly. */
void stopA()
{
  stop(motorPinA1, motorPinA2);
}

/** @brief Stop actuator B softly. */
void stopB()
{
  stop(motorPinB1, motorPinB2);
}

/**
 * @brief PWM forward helper for a motor pair.
 * @param Pin1 H-bridge IN1.
 * @param Pin2 H-bridge IN2.
 * @param speed PWM duty (0-255).
 */
void forward(int Pin1, int Pin2, int speed)
{
  analogWrite(Pin1, speed);
  analogWrite(Pin2, 0);
}

/**
 * @brief PWM reverse helper for a motor pair.
 * @param Pin1 H-bridge IN1.
 * @param Pin2 H-bridge IN2.
 * @param speed PWM duty (0-255).
 */
void reverse(int Pin1, int Pin2, int speed)
{
  analogWrite(Pin1, 0);
  analogWrite(Pin2, speed);
}

/**
 * @brief Soft stop both pins (0% duty).
 * @param motorPin1 H-bridge IN1.
 * @param motorPin2 H-bridge IN2.
 */
void stop(int motorPin1, int motorPin2)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
}

/**
 * @brief Hard stop by driving both pins HIGH briefly.
 * @param motorPin1 H-bridge IN1.
 * @param motorPin2 H-bridge IN2.
 */
void stopSuddenly(int motorPin1, int motorPin2)
{
  analogWrite(motorPin1, 255);
  analogWrite(motorPin2, 255);
}

// 従来版
/** @brief Legacy digital forward for actuator A. */
void forwardDigitalA()
{
  forwardDigital(motorPinA1, motorPinA2);
}

/** @brief Legacy digital forward for actuator B. */
void forwardDigitalB()
{
  forwardDigital(motorPinB1, motorPinB2);
}

/** @brief Legacy digital reverse for actuator A. */
void reverseDigitalA()
{
  reverseDigital(motorPinA1, motorPinA2);
}

/** @brief Legacy digital reverse for actuator B. */
void reverseDigitalB()
{
  reverseDigital(motorPinB1, motorPinB2);
}

/**
 * @brief Digital forward helper (no PWM).
 * @param Pin1 H-bridge IN1.
 * @param Pin2 H-bridge IN2.
 */
void forwardDigital(int Pin1, int Pin2)
{
  digitalWrite(Pin1, HIGH);
  digitalWrite(Pin2, LOW);
}

/**
 * @brief Digital reverse helper (no PWM).
 * @param Pin1 H-bridge IN1.
 * @param Pin2 H-bridge IN2.
 */
void reverseDigital(int Pin1, int Pin2)
{
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, HIGH);
}
