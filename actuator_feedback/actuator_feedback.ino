// 可変抵抗は時計回りを角度が増加する方向となるに接続すること

const int motorPinA1 = 4;  // アクチュエータA 用 IN1
const int motorPinA2 = 5;  // アクチュエータA 用 IN2
const int motorPinB1 = 15; // アクチュエータB 用 IN1
const int motorPinB2 = 2;  // アクチュエータB 用 IN2

const int potentio_SOCKET_FB = 34; // 可変抵抗器のアナログ入力ソケット番号
const int potentio_SOCKET_LR = 35; // 可変抵抗器のアナログ入力ソケット番号

const int SPEED_FAST = 255;
const int SPEED_SLOW = 255 / 3;

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

void setup()
{
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(potentio_SOCKET_FB, INPUT);
  pinMode(potentio_SOCKET_LR, INPUT);

  Serial.begin(9600);
  Serial.println("Program started.");

  delay(5000);
  // 角度の初期値（水平時の値）を取得
  resetinitialAngleFB();
  resetinitialAngleLR();
}

void loop()
{
  getAngleFB();
  getAngleLR();
  printAngle();
  if (Serial.available() > 0)
  {
    char c = Serial.read();

    // 改行コードは無視
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
    // 低速動作用
    case 'W': // 前相当
      forwardSlowA();
      reverseSlowB();
      break;

    case 'S': // 後ろ相当
      reverseSlowA();
      forwardSlowB();
      break;

    case 'A': // 左相当
      // moveToAngle(20); // つまみが時計回り20度の傾き
      reverseSlowA();
      reverseSlowB();
      break;

    case 'D': // 右相当
      // moveToAngle(-20); // つまみが反時計回り20度の傾き
      forwardSlowA();
      forwardSlowB();
      break;

    case 'X': // 前後方向を水平にする
      moveToAngleFrontBack(0);
      break;

    case 'Z': // 左右方向を水平にする
      moveToAngleLeftRight(0);
      break;

    // 高速動作用
    case 'U': // 前相当
      forwardFastA();
      reverseFastB();
      break;

    case 'J': // 後ろ相当
      reverseFastA();
      forwardFastB();
      break;

    case 'H': // 左相当
      reverseFastA();
      reverseFastB();
      break;

    case 'K': // 右相当
      forwardFastA();
      forwardFastB();
      break;

    case 'O': // 前後に振動
      moveFrontBack(10);
      break;

    case 'P': // 左右に振動
      moveLeftRight(10);
      break;

    case 'G': // ガタンゴトン動作
      moveGatanGoton(10);
      break;

    case 'R':
      resetinitialAngleFB();
      resetinitialAngleLR();
      break;

    default: // その他キー → 停止
      stopA();
      stopB();
      Serial.print("(Unknown key)");
      break;
    }
    Serial.println();
  }
}

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
void moveFrontBack(int times)
{
  reverseSlowA();
  forwardSlowB();
  delay(250);
  stopA();
  stopB();
  for (int count = 1; count <= times; count++)
  {
    forwardSlowA(); // 台時計回り
    reverseSlowB();
    delay(400);
    stopA();
    stopB();
    reverseSlowA(); // 台時計回り
    forwardSlowB();
    delay(500);
    stopA();
    stopB();
  }
  stopA();
  stopB();
  forwardSlowA();
  reverseSlowB();
  delay(250);
  stopA();
  stopB();
  return;
}

// 左右に振動
void moveLeftRight(int times)
{
  delay(100);
  forwardSlowA();
  forwardSlowB();
  delay(400);
  stopA();
  stopB();
  for (int count = 1; count <= times; count++)
  {
    reverseSlowA();
    reverseSlowB();
    delay(770);
    stopA();
    stopB();
    forwardSlowA();
    forwardSlowB();
    delay(800);
    stopA();
    stopB();
  }
  stopA();
  stopB();
  reverseSlowA();
  reverseSlowB();
  delay(400);
  stopA();
  stopB();
  return;
}

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
void moveToAngleFrontBack(int targetRelativeAngle)
{
  Serial.print("move to ");
  Serial.println(targetRelativeAngle);

  int targetAngleFB = initialAngleFB + targetRelativeAngle; // 目標の絶対角度(可変抵抗の角度) を計算

  while (1)
  {
    Serial.print("initialAngleFB: ");
    Serial.print(initialAngleFB);
    Serial.print("\t targetRelativeAngleFB: ");
    Serial.print(targetAngleFB);
    Serial.print("\t currentAngleFB: ");
    int currentAngle = getAngleFB();
    Serial.println(currentAngle);

    if (TOLERANCE < currentAngle - targetAngleFB)
    {
      Serial.println("つまみ反時計回り");
      forwardSlowA();
      reverseSlowB();
    }
    else if (TOLERANCE < targetAngleFB - currentAngle)
    {
      Serial.println("つまみ時計回り");
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
void moveToAngleLeftRight(int targetRelativeAngle)
{
  Serial.print("move to ");
  Serial.println(targetRelativeAngle);

  int targetAngleLR = initialAngleLR + targetRelativeAngle; // 目標の絶対角度(可変抵抗の角度) を計算

  while (1)
  {
    Serial.print("initialAngleLR: ");
    Serial.print(initialAngleLR);
    Serial.print("\t targetRelativeAngleLR: ");
    Serial.print(targetAngleLR);
    Serial.print("\t currentAngleLR: ");
    int currentAngle = getAngleLR();
    Serial.println(currentAngle);

    if (TOLERANCE < currentAngle - targetAngleLR)
    {
      Serial.println("つまみ時計回り");
      forwardSlowA();
      forwardSlowB();
    }
    else if (TOLERANCE < targetAngleLR - currentAngle)
    {
      Serial.println("つまみ反時計回り");
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

int getAnalogInputFB()
{
  return analogRead(potentio_SOCKET_FB);
}

int getAnalogInputLR()
{
  return analogRead(potentio_SOCKET_LR);
}

void forwardFastA()
{
  forward(motorPinA1, motorPinA2, SPEED_FAST);
}

void forwardFastB()
{
  forward(motorPinB1, motorPinB2, SPEED_FAST);
}

void reverseFastA()
{
  reverse(motorPinA1, motorPinA2, SPEED_FAST);
}

void reverseFastB()
{
  reverse(motorPinB1, motorPinB2, SPEED_FAST);
}

void forwardSlowA()
{
  forward(motorPinA1, motorPinA2, SPEED_SLOW);
}

void forwardSlowB()
{
  forward(motorPinB1, motorPinB2, SPEED_SLOW);
}

void reverseSlowA()
{
  reverse(motorPinA1, motorPinA2, SPEED_SLOW);
}

void reverseSlowB()
{
  reverse(motorPinB1, motorPinB2, SPEED_SLOW);
}

void stopA()
{
  stop(motorPinA1, motorPinA2);
}

void stopB()
{
  stop(motorPinB1, motorPinB2);
}

void forward(int Pin1, int Pin2, int speed)
{
  analogWrite(Pin1, speed);
  analogWrite(Pin2, 0);
}

void reverse(int Pin1, int Pin2, int speed)
{
  analogWrite(Pin1, 0);
  analogWrite(Pin2, speed);
}

void stop(int motorPin1, int motorPin2)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
}

void stopSuddenly(int motorPin1, int motorPin2)
{
  analogWrite(motorPin1, 255);
  analogWrite(motorPin2, 255);
}

// 従来版
void forwardDigitalA()
{
  forwardDigital(motorPinA1, motorPinA2);
}

void forwardDigitalB()
{
  forwardDigital(motorPinB1, motorPinB2);
}

void reverseDigitalA()
{
  reverseDigital(motorPinA1, motorPinA2);
}

void reverseDigitalB()
{
  reverseDigital(motorPinB1, motorPinB2);
}

void forwardDigital(int Pin1, int Pin2)
{
  digitalWrite(Pin1, HIGH);
  digitalWrite(Pin2, LOW);
}

void reverseDigital(int Pin1, int Pin2)
{
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, HIGH);
}
