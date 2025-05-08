#include "medic_device.h"

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MPU6050 mpu;

double tempReading = 0;

// Get readings from temperature sensor
void handleTemperature(void *pvParameters) {

  tempReading = mlx.readObjectTempC();
  Serial.print("Object = ");
  Serial.print(tempReading);
  Serial.println("*C");

  vTaskDelay(2000 / portTICK_PERIOD_MS);
}

// Get readings from accelerometer and count the number of steps
void countSteps(void *pvParameters) {

}

// Get readings from the heart rate/oxygen level sensor and calculate the blood pressure
void bloodPressure(void *pvParameters) {

}

// Display to the oLED screen
void handleDisplay(void *pvParameters) {

}

// Connect to IoT platform adnd update the platform
void BlynkHandler(void *pvParameters) {

  while (1) {
    if (Blynk.connected()) {
      Blynk.run();

    } else {
      // Try reconnecting if not connected
      Blynk.connect();
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void wifiBlynkManagerTask(void *pvParameters) {
  while (1) {
    // If WiFi is not connected, reconnect
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Disconnected! Trying to reconnect...");
      WiFi.disconnect();
      WiFi.begin(ssid, pass);

      // Wait for connection for up to 10 seconds
      int retries = 0;
      while (WiFi.status() != WL_CONNECTED && retries < 20) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        retries++;
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] Reconnected successfully.");
      } else {
        Serial.println("\n[WiFi] Failed to reconnect.");
      }
    }

    // If WiFi is connected but Blynk is not connected
    if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
      Serial.println("[Blynk] Disconnected! Trying to reconnect...");
      Blynk.connect();
      if (Blynk.connected()) {
        Serial.println("[Blynk] Reconnected successfully.");
      } else {
        Serial.println("[Blynk] Failed to reconnect.");
      }
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Check every 5 seconds
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");
}

void loop() {
  // put your main code here, to run repeatedly:

}
