// ESP32 Dev Module + Ticker でエンコーダを周期読み取り
#include <Ticker.h>
#include <Encoder.h>

// エンコーダのピン（ESP32 で安全な入力ピン例）
const int pinA = 32;
const int pinB = 33;
Encoder myEnc(pinA, pinB);

// 読み取り用の共有変数
volatile long oldPosition = -999;
volatile long newPosition = 0;

// 生のピン状態デバッグ用
volatile int lastA = -1;
volatile int lastB = -1;

// 3ms 周期でフラグを立てる Ticker
Ticker ticker;
volatile bool encTick = false;

void onTick() {
  // コールバックは軽く保ち、処理は loop() 側で行う
  encTick = true;
}

void setup() {
  Serial.begin(115200);
  // エンコーダ入力は内部プルアップを有効化（外部配線に応じて変更）
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  // 初期状態を読み取り
  lastA = digitalRead(pinA);
  lastB = digitalRead(pinB);
  Serial.print("Init A,B: ");
  Serial.print(lastA);
  Serial.print(", ");
  Serial.println(lastB);
  // 3ms 周期で呼び出し
  ticker.attach_ms(3, onTick);
}

void loop() {
  // フラグが立っていたらエンコーダ値を取得
  if (encTick) {
    encTick = false;
    newPosition = myEnc.read();
  }

  // 変化があれば表示
  if (newPosition != oldPosition) {
    long delta = newPosition - oldPosition;
    oldPosition = newPosition;
    if (delta > 0) {
      Serial.print("CW  Angle: ");
      Serial.println(newPosition);
    } else {
      Serial.print("CCW Angle: ");
      Serial.println(newPosition);
    }
  }

  // A/Bの生状態が変化したら表示（配線確認用）
  int a = digitalRead(pinA);
  int b = digitalRead(pinB);
  if (a != lastA || b != lastB) {
    lastA = a;
    lastB = b;
    Serial.print("A,B change -> ");
    Serial.print(a);
    Serial.print(", ");
    Serial.println(b);
  }
}
