#include <Arduino.h>
#include <freertos/queue.h>

// ESP32 dual-core assignment (core0 = PRO_CPU, core1 = APP_CPU).
constexpr BaseType_t ACTUATOR_CORE = 1;
constexpr BaseType_t RELAY_CORE = 0;

TaskHandle_t actuatorTaskHandle = nullptr;
TaskHandle_t relayTaskHandle = nullptr;
QueueHandle_t gActuatorCommandQueue = nullptr;
QueueHandle_t gRelayCommandQueue = nullptr;

void actuatorTask(void *pvParameters);
void relayTask(void *pvParameters);
void routeSerialCommands();
void runActuatorLoop();
void runRelayLoop();
void setupActuator();
void setupRelay();

/** @brief Arduino entry: initialize serial, queues, drivers, and start tasks. */
void setup()
{
  Serial.begin(9600);
  delay(200);

  gActuatorCommandQueue = xQueueCreate(16, sizeof(char));
  gRelayCommandQueue = xQueueCreate(16, sizeof(char));

  if (gActuatorCommandQueue == nullptr || gRelayCommandQueue == nullptr)
  {
    Serial.println("Failed to allocate command queues.");
    while (true)
    {
      delay(1000);
    }
  }

  setupActuator();
  setupRelay();

  xTaskCreatePinnedToCore(actuatorTask, "ActuatorMain", 8192, nullptr, 3, &actuatorTaskHandle, ACTUATOR_CORE);
  xTaskCreatePinnedToCore(relayTask, "RelaySub", 4096, nullptr, 2, &relayTaskHandle, RELAY_CORE);

  Serial.println("multi_task initialized. Use actuator commands as before.");
  Serial.println("Prefix relay commands with 'R' then the relay key (e.g. R O, R K).");
}

/** @brief Poll serial and forward commands into the proper task queue. */
void loop()
{
  routeSerialCommands();
  delay(1);
}

/** @brief Task wrapper to run the actuator main loop on the APP core. */
void actuatorTask(void *pvParameters)
{
  while (true)
  {
    runActuatorLoop();
    delay(1);
  }
}

/** @brief Task wrapper to run the relay loop on the PRO core. */
void relayTask(void *pvParameters)
{
  while (true)
  {
    runRelayLoop();
    delay(1);
  }
}

/** @brief Read serial characters and dispatch to actuator or relay queue. */
void routeSerialCommands()
{
  static bool relayMode = false;

  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\r' || c == '\n')
    {
      continue;
    }

    if (relayMode)
    {
      xQueueSend(gRelayCommandQueue, &c, 0);
      relayMode = false;
      continue;
    }

    if (c == 'R' || c == 'r')
    {
      relayMode = true;
      continue;
    }

    // Default: send to actuator task.
    xQueueSend(gActuatorCommandQueue, &c, 0);
  }
}
