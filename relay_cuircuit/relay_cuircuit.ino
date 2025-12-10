#define RELAY_PIN 14

void setup()
{
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Ready to receive commands.");
}

void loop()
{
  if (Serial.available() > 0)
  {
    char c = Serial.read();

    // 改行コードは無視
    if (c == '\r' || c == '\n')
      return;

    Serial.print("Command: ");
    Serial.print(c);

    switch (c)
    {
    case 'P':
      Serial.print("(ON)");
      digitalWrite(RELAY_PIN, HIGH);
      break;

    case 'L':
      Serial.print("(OFF)");
      digitalWrite(RELAY_PIN, LOW);
      break;

    default:
      Serial.print("(Unknown key)");
      break;
    }
    Serial.println();
  }
}