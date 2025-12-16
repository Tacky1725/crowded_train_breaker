#include <freertos/queue.h>

#define RELAY_PIN_1 12
#define RELAY_PIN_2 14

extern QueueHandle_t gRelayCommandQueue;

/** @brief Initialize relay pins and set both OFF. */
void setupRelay()
{
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  Serial.println("Relay Program started.");
}

/** @brief Process one queued relay command. */
void runRelayLoop()
{
  if (gRelayCommandQueue == nullptr)
    return;

  char c;
  if (xQueueReceive(gRelayCommandQueue, &c, 0) != pdTRUE)
    return;

  if (c == '\r' || c == '\n')
    return;

  Serial.print("Relay Command: ");
  Serial.print(c);

  switch (c)
  {
  case 'O':
    Serial.print("(1-ON)");
    digitalWrite(RELAY_PIN_1, HIGH);
    break;

  case 'K':
    Serial.print("(1-OFF)");
    digitalWrite(RELAY_PIN_1, LOW);
    break;

  case 'P':
    Serial.print("(2-ON)");
    digitalWrite(RELAY_PIN_2, HIGH);
    break;

  case 'L':
    Serial.print("(2-OFF)");
    digitalWrite(RELAY_PIN_2, LOW);
    break;

  case 'I':
    Serial.print("(ALL-ON)");
    digitalWrite(RELAY_PIN_1, HIGH);
    digitalWrite(RELAY_PIN_2, HIGH);
    break;

  case 'J':
    Serial.print("(ALL-OFF)");
    digitalWrite(RELAY_PIN_1, LOW);
    digitalWrite(RELAY_PIN_2, LOW);
    break;

  default:
    Serial.print("(Unknown key)");
    digitalWrite(RELAY_PIN_1, LOW);
    digitalWrite(RELAY_PIN_2, LOW);
    break;
  }
  Serial.println();
}
