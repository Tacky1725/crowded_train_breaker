TaskHandle_t thp[1]; // マルチスレッドのタスクハンドル格納用

void setup()
{
  Serial.begin(9600);
  xTaskCreatePinnedToCore(SubCore, "SubCore", 4096, NULL, 3, &thp[0], 0);
}

void loop()
{
  Serial.println(getA()); // メインで実行するプログラム
  delay(100);
}

void SubCore(void *args)
{
  while (1)
  {
    delay(1); // サブで実行するプログラムを書く
    incrementA();
  }
}
