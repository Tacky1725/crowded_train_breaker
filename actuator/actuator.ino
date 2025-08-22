/*
  DCモーターを制御するプログラム (シリアル出力付き)
  - 3秒間 正転
  - 3秒間 停止
  - 3秒間 逆転
  - 3秒間 停止
  上記の動作を繰り返し、各動作をシリアルモニターに出力します。
*/

// モータードライバーに接続するピンを定義
const int motorPinA1 = 5;  // モータードライバーのIN1に接続
const int motorPinA2 = 6;  // モータードライバーのIN2に接続
const int motorPinB1 = 10;  // モータードライバーのIN1に接続
const int motorPinB2 = 11;  // モータードライバーのIN2に接続
boolean isMove = true;
int count = 0;

void setup() {
  // 2つのピンを出力モードに設定
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);

  // シリアル通信を初期化 (ボーレート: 9600 bps)
  // この命令で、ArduinoがPCと通信できるようになります。
  Serial.begin(9600);
  Serial.println("モーター制御プログラムを開始します。");
}

void loop() {
  while (count < 5) {
    forward(motorPinA1, motorPinA2);
    delay(3000);
    forward(motorPinB1, motorPinB2);
    delay(5000);
    stop(motorPinA1, motorPinA2);
    stop(motorPinB1, motorPinB2);
    delay(3000);
    reverse(motorPinA1, motorPinA2);
    delay(3000);
    reverse(motorPinB1, motorPinB2);
    delay(5000);
    stop(motorPinA1, motorPinA2);
    stop(motorPinB1, motorPinB2);
    delay(3000);
    count++;
  }
}

void forward(int motorPin1, int motorPin2) {
  // 正転 (Forward)
  Serial.print(motorPin1);
  Serial.println(" Forward");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}

void reverse(int motorPin1, int motorPin2) {
  // 逆転 (Reverse)
  Serial.print(motorPin1);
  Serial.println(" Reverse");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
}

void stop(int motorPin1, int motorPin2) {
  // 停止 (Stop/Brake)
  Serial.print(motorPin1);
  Serial.println(" Stop");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}