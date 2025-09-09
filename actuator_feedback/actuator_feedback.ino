// 可変抵抗は時計回りを角度が増加する方向となるに接続すること

// モータードライバ（L9110S）ーに接続するピンを定義
const int motorPinA1 = 25; // アクチュエータA 用 IN1
const int motorPinA2 = 26; // アクチュエータA 用 IN2
const int motorPinB1 = 32; // アクチュエータB 用 IN1
const int motorPinB2 = 33; // アクチュエータB 用 IN2

const int motorPinPwmA = 3; // アクチュエータA 用 PWMピン
const int motorPinPwmB = 5; // アクチュエータB 用 PWMピン

int potentio_SOCKET = 15; // 可変抵抗器のアナログ入力ソケット番号

const int SPEED = 255;

int initialAngle = 0; // 角度の初期値（水平時の値）

int stopFlag = 0;

void setup()
{
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);

  // pinMode(motorPinPwmA, OUTPUT);
  // pinMode(motorPinPwmB, OUTPUT);

  Serial.begin(9600);
  Serial.println("W/S/A/D でアクチュエータ制御（W:上, S:下, A:左, D:右, 他: 停止）");

  // 角度の初期値（水平時の値）を取得

  delay(5000);
  int initialAngle1 = getAngle();
  delay(1000);
  int initialAngle2 = getAngle();
  delay(1000);
  int initialAngle3 = getAngle();
  initialAngle = (initialAngle1 + initialAngle2 + initialAngle3) / 3; // 平均値を取る

  Serial.print("Initial Angle: ");
  Serial.println(initialAngle);
  Serial.println("Ready to receive commands.");
}

void loop()
{
  // getAngle();
  if (Serial.available() > 0)
  {
    char c = Serial.read();

    // 改行コードは無視
    if (c == '\r' || c == '\n')
      return;

    stopA();
    stopB();
    delay(100);

    switch (c)
    {
    // フィードバック制御用
    case 'W': // 上相当 → 両方縮める (reverse)
      // フィードバック処理に要修正
      reverseA();
      reverseB();
      break;

    case 'S': // 下相当 → 両方伸ばす (forward)
      // フィードバック処理に要修正
      forwardA();
      forwardB();
      break;

    case 'A':          // 左相当 → A縮める, B伸ばす
      moveToAngle(20); // つまみが時計回り20度の傾き
      break;

    case 'D':           // 右相当 → A伸ばす, B縮める
      moveToAngle(-20); // つまみが反時計回り20度の傾き
      break;

    case 'X': // 左右方向を水平にする
      moveToAngle(0);
      break;

    case 'C': // 上下方向を水平にする
      // 要作成
      break;

    // 手動制御用
    case 'U': // 上相当 → 両方縮める (reverse)
      reverseA();
      reverseB();
      Serial.println("Command: U (Both Reverse)");
      break;

    case 'J': // 下相当 → 両方伸ばす (forward)
      forwardA();
      forwardB();
      Serial.println("Command: J (Both Forward)");
      break;

    case 'H': // 左相当 → A縮める, B伸ばす
      reverseA();
      forwardB();
      Serial.println("Command: H (reverse A, forward B)");
      break;

    case 'K': // 右相当 → A伸ばす, B縮める
      forwardA();
      reverseB();
      Serial.println("Command: K (forward A, reverse B)");
      break;

    case 'L': // 前後に振動
      moveFrontBack(10);
      break;

    case 'P': // 左右に振動
      moveLeftRight(10);
      break;

    case 'G': // ガタンゴトン動作
      moveGatanGoton(10);
      break;

    default: // その他キー → 停止
      stopA();
      stopB();
      Serial.print("Unknown key: ");
      Serial.println(c);
      break;
    }
  }
}

void moveLeftRight(int times)
{
  reverseA();
  forwardB();
  delay(100);
  stopA();
  stopB();
  for (int count = 1; count <= times; count++)
  {
    forwardA(); // 台時計回り
    reverseB();
    delay(200);
    stopA();
    stopB();
    reverseA(); // 台時計回り
    forwardB();
    delay(250);
    stopA();
    stopB();
  }
  stopA();
  stopB();
  forwardA();
  reverseB();
  delay(100);
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
    forwardA();
    forwardB();
    delay(120);
    stopA();
    stopB();
    reverseA();
    reverseB();
    delay(150);
    stopA();
    stopB();
    delay(1000);
  }
  stopA();
  stopB();
  return;
}

void moveFrontBack(int times)
{
  delay(100);
  forwardA();
  forwardB();
  delay(100);
  stopA();
  stopB();
  for (int count = 1; count <= times; count++)
  {
    reverseA();
    reverseB();
    delay(150);
    stopA();
    stopB();
    forwardA();
    forwardB();
    delay(180);
    stopA();
    stopB();
  }
  stopA();
  stopB();
  reverseA();
  reverseB();
  delay(100);
  stopA();
  stopB();
  return;
}

void rotateForward()
{
}

// 角度を指定して動かす
void moveToAngle(int targetRelativeAngle)
{
  Serial.print("move to ");
  Serial.println(targetRelativeAngle);
  int tolerance = 5; // 許容誤差

  int targetAngle = initialAngle + targetRelativeAngle; // 目標の絶対角度(可変抵抗の角度) を計算

  while (1)
  {
    Serial.print("initialAngle: ");
    Serial.print(initialAngle);
    Serial.print("\t targetRelativeAngle: ");
    Serial.print(targetAngle);
    Serial.print("\t currentAngle: ");
    int currentAngle = getAngle();
    Serial.println(currentAngle);

    if (tolerance < currentAngle - targetAngle)
    {
      Serial.println("つまみ反時計回り");
      forwardA();
      reverseB();
    }
    else if (tolerance < targetAngle - currentAngle)
    {
      Serial.println("つまみ時計回り");
      reverseA();
      forwardB();
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
      int randomInt = random(-15, 15); // 新しい目標角度を設定
      Serial.print("randomInt: ");
      Serial.println(randomInt);
      moveToAngle(randomInt);
    }
  }
  stopA();
  stopB();
}

int getAngle()
{
  int analogin; // アナログ入力値を代入する変数
  // analogin = getAnalogInput();  // アナログ入力ソケット入力値を変数に代入
  // アナログ入力の平均を取るように変更
  int analogin1 = getAnalogInput();
  delay(5);
  int analogin2 = getAnalogInput();
  delay(5);
  int analogin3 = getAnalogInput();
  analogin = (analogin1 + analogin2 + analogin3) / 3;
  // ここまで変更内容

  int angle = map(analogin, 0, 4095, 0, 270);      // アナログ入力レンジ => 角度指示レンジに変換
  int percentage = map(analogin, 0, 4095, 0, 100); // アナログ入力レンジ => アナログ出力レンジに変換

  // Serial.print("initialAngle: ");
  // Serial.print(initialAngle);
  // Serial.print(", analogin: ");
  // Serial.print(analogin);
  // Serial.print(", Angle: ");
  // Serial.println(angle);

  return angle;
}

int getAnalogInput()
{
  return analogRead(potentio_SOCKET);
}

void forwardA()
{
  forward(motorPinA1, motorPinA2, motorPinPwmA);
}

void forwardB()
{
  forward(motorPinB1, motorPinB2, motorPinPwmB);
}

void reverseA()
{
  reverse(motorPinA1, motorPinA2, motorPinPwmA);
}

void reverseB()
{
  reverse(motorPinB1, motorPinB2, motorPinPwmB);
}

void stopA()
{
  stop(motorPinA1, motorPinA2);
}

void stopB()
{
  stop(motorPinB1, motorPinB2);
}

// 引数はモーター番号のみに変更
void forward(int Pin1, int Pin2, int PinPwm)
{
  digitalWrite(Pin1, HIGH);
  digitalWrite(Pin2, LOW);

  // analogWrite(motorPinPwm, SPEED);
}

void reverse(int Pin1, int Pin2, int PinPwm)
{
  digitalWrite(Pin1, LOW);
  digitalWrite(Pin2, HIGH);

  // analogWrite(motorPinPwm, SPEED);
}

void stop(int motorPin1, int motorPin2)
{
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void stopSuddenly(int motorPin1, int motorPin2)
{
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, HIGH);
}
