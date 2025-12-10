// ロータリエンコーダの角度計算プログラム
const int pinA = 32;  // A相のピン
const int pinB = 33;  // B相のピン

// エンコーダの状態管理用変数
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile long lastEncoderValue = 0;

// エンコーダ1回転あたりのパルス数（エンコーダの仕様に合わせて変更）
const int PULSES_PER_REV = 360;

void setup() {
  Serial.begin(115200);
  
  // エンコーダピンを入力として設定（プルアップ有効）
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  
  // 初期状態を読み取り
  lastEncoded = (digitalRead(pinA) << 1) | digitalRead(pinB);
  
  Serial.println("Rotary Encoder Angle Measurement Start");
}

void loop() {
  // 現在のA相とB相の状態を読み取り
  int MSB = digitalRead(pinA);
  int LSB = digitalRead(pinB);
  
  // 現在の状態を2ビットの数値にエンコード
  int encoded = (MSB << 1) | LSB;
  
  // 前回の状態と現在の状態から回転を判定
  int sum = (lastEncoded << 2) | encoded;
  
  // 回転方向に応じてカウンタを更新
  switch(sum) {
    case 0b1101:
    case 0b0100:
    case 0b0010:
    case 0b1011:
      encoderValue++;
      break;
    case 0b1110:
    case 0b0111:
    case 0b0001:
    case 0b1000:
      encoderValue--;
      break;
  }
  
  lastEncoded = encoded;
  
  // 値が変化した場合のみ角度を計算して表示
  if(encoderValue != lastEncoderValue) {
    // 角度を計算（エンコーダ値から360度への変換）
    float angle = (float)encoderValue * 360.0 / (PULSES_PER_REV * 4);
    
    Serial.print("Encoder Value: ");
    Serial.print(encoderValue);
    Serial.print("  Angle: ");
    Serial.print(angle);
    Serial.println(" degrees");
    
    lastEncoderValue = encoderValue;
  }
  
  // チャタリング防止のための短い遅延
  delay(1);
}