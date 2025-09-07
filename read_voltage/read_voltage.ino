int potentio_SOCKET = 15;  //可変抵抗器のアナログ入力ソケット番号

void setup() {
  //シリアル通信設定
  Serial.begin(9600);  //シリアル通信速度設定
}

void loop() {
  Serial.println(getAnalogInput());  //回転角度指示値をシリアル出力
  delay(50);                 //0.05秒ウェイト
}

int getAngle() {
  int analogin;                            //アナログ入力値を代入する変数
  analogin = getAnalogInput();  //アナログ入力ソケット入力値を変数に代入

  int angle = map(analogin, 0, 4095, 0, 270);       //アナログ入力レンジ => 角度指示レンジに変換
  int percentage = map(analogin, 0, 4095, 0, 100);  //アナログ入力レンジ => アナログ出力レンジに変換

  return angle;
}

int getAnalogInput() {
  return analogRead(potentio_SOCKET);
}