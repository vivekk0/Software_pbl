#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// Pin Definitions
#define DHTPIN 4
#define DHTTYPE DHT11
#define AIR_SENSOR_PIN 34

// Sensor Setup
DHT dht(DHTPIN, DHTTYPE);

// Task Handles
TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Let serial settle

  dht.begin();

  // Create DHT Sensor Task (Core 0)
  xTaskCreatePinnedToCore(
    readDHTTask,
    "DHT Task",
    4096,      // Increased stack size
    NULL,
    1,
    &Task1,
    0);

  // Create Air Quality Sensor Task (Core 1)
  xTaskCreatePinnedToCore(
    readAirQualityTask,
    "Air Quality Task",
    2048,
    NULL,
    1,
    &Task2,
    1);
}

// DHT Sensor Task
void readDHTTask(void *parameter) {
  for (;;) {
    // Delay before reading (avoid sensor timing issues)
    vTaskDelay(100 / portTICK_PERIOD_MS);

    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
      Serial.println("[DHT] Failed to read from sensor!");
    } else {
      Serial.printf("[DHT] Temp: %.2f°C, Humidity: %.2f%%\n", temp, hum);
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Wait 2 seconds
  }
}

// Air Quality Task
void readAirQualityTask(void *parameter) {
  for (;;) {
    int rawVal = analogRead(AIR_SENSOR_PIN);
    float voltage = rawVal * (3.3 / 4095.0);  // Convert to voltage

    Serial.printf("[AIR] Raw: %d, Voltage: %.2f V\n", rawVal, voltage);

    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Wait 3 seconds
  }
}

void loop() {
  // Nothing here – all tasks are handled on cores
}