// モータードライバ（L9110S）ーに接続するピンを定義
const int motorPinA1 = 25;   // アクチュエータA 用 IN1
const int motorPinA2 = 26;   // アクチュエータA 用 IN2
const int motorPinB1 = 32;  // アクチュエータB 用 IN1
const int motorPinB2 = 33;  // アクチュエータB 用 IN2

const int motorPinPwmA = 3;  // アクチュエータA 用 PWMピン
const int motorPinPwmB = 5;  // アクチュエータB 用 PWMピン
const int SPEED = 255;

void setup() {
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);

  // pinMode(motorPinPwmA, OUTPUT);
  // pinMode(motorPinPwmB, OUTPUT);

  Serial.begin(9600);
  Serial.println("W/S/A/D でアクチュエータ制御（W:上, S:下, A:左, D:右, 他: 停止）");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    // 改行コードは無視
    if (c == '\r' || c == '\n') return;

    stopA();
    stopB();
    delay(100);

    switch (c) {
      case 'W':  // 上相当 → 両方伸ばす (forward)
        forwardA();
        forwardB();
        Serial.println("Command: W (Both Forward)");
        break;

      case 'S':  // 下相当 → 両方縮める (reverse)
        reverseA();
        reverseB();
        Serial.println("Command: S (Both Reverse)");
        break;

      case 'A':  // 左相当 → A縮める, B伸ばす
        reverseA();
        forwardB();
        Serial.println("Command: A (A Reverse, B Forward)");
        break;

      case 'D':  // 右相当 → A伸ばす, B縮める
        forwardA();
        reverseB();
        Serial.println("Command: D (A Forward, B Reverse)");
        break;

      default:  // その他キー → 停止
        stopA();
        stopB();
        Serial.print("Unknown key: ");
        Serial.println(c);
        break;
    }
  }
}

void forwardA() {
  forward(motorPinA1, motorPinA2, motorPinPwmA);
}

void forwardB() {
  forward(motorPinB1, motorPinB2, motorPinPwmB);
}

void reverseA() {
  reverse(motorPinA1, motorPinA2, motorPinPwmA);
}

void reverseB() {
  reverse(motorPinB1, motorPinB2, motorPinPwmB);
}

void stopA() {
  stop(motorPinA1, motorPinA2);
}

void stopB() {
  stop(motorPinB1, motorPinB2);
}

// 引数はモーター番号のみに変更
void forward(int motorPin1, int motorPin2, int motorPinPwm) {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  // analogWrite(motorPinPwm, SPEED);
}

void reverse(int motorPin1, int motorPin2, int motorPinPwm) {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);

  // analogWrite(motorPinPwm, SPEED);
}

void stop(int motorPin1, int motorPin2) {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void stopSuddenly(int motorPin1, int motorPin2) {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, HIGH);
}
