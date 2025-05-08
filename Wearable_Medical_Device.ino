#include "medic_device.h"

#define I2C_SDA 8
#define I2C_SCL 9
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

double tempReading = 0.0;
double systolic = 0.0;
double diastolic = 0.0;
int stepCount = 0;

unsigned long lastStepTime = 0;

#define STEP_DELAY 250 //in ms
#define THRESHOLD 9.8

portMUX_TYPE taskMux = portMUX_INITIALIZER_UNLOCKED; 

// Get readings from temperature sensor
void handleTemperature(void *pvParameters) {

  while (1) { 
    tempReading = mlx.readObjectTempC();
    Serial.print("Object = ");
    Serial.print(tempReading);
    Serial.println("*C");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

//Get readings from accelerometer and count the number of steps
void countSteps(void *pvParameters) {
  sensors_event_t a, g, temp;
  float ax, ay, az, alpha = 0.2;
  float magA, filtA = 0;

  while (1) {
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;

    Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(a.acceleration.z);
      Serial.println(" m/s^2");

    magA = sqrt(ax * ax + ay * ay);

    // filter

    unsigned long now = millis();
    if ((magA > THRESHOLD) && (now - lastStepTime) > STEP_DELAY) {
      stepCount++;
      lastStepTime = now;
    }

    Serial.print(now);
    Serial.print(",");
    Serial.print(filtA, 2);
    Serial.print(",");
    Serial.println(stepCount);

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Get readings from the heart rate/oxygen level sensor and calculate the blood pressure
void bloodPressure(void *pvParameters) {
  // Sensor setup

  Wire.begin(8, 9);
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Check wiring.");
    vTaskDelete(NULL); // kill the task
  }

  particleSensor.setup(100, 4, 2, 100, 411, 4096); // sampleRate, avg, pulseWidth, LED power, etc.
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  while (1) {
    // Collect 100 samples
    int i = 0;
    while (i < 100) {
      if (particleSensor.available()) {
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
        i++;
      } else {
        particleSensor.check(); // manually poll if necessary
      }
      vTaskDelay(pdMS_TO_TICKS(10)); // sample every 10ms → 100Hz
    }

    // Call Maxim algorithm
    maxim_heart_rate_and_oxygen_saturation(irBuffer, 100,
                                           redBuffer,
                                           &spo2, &validSPO2,
                                           &heartRate, &validHeartRate);

    // Print results
    if (validHeartRate && validSPO2) {
      Serial.print("HR: "); Serial.print(heartRate);
      Serial.print(" bpm | SpO2: "); Serial.print(spo2);
      Serial.println(" %");
      portENTER_CRITICAL(&taskMux);
      diastolic = ((3.6 - 0.04 * spo2) / 0.012) + 60;
      systolic = ((3.4 - 0.04 * spo2) / 0.012) + 50;
      portEXIT_CRITICAL(&taskMux);
    } else {
      Serial.println("Invalid HR or SpO2 reading.");
    }

    // Delay before next reading cycle
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2-second pause
  }
}

// Display to the oLED screen
void handleDisplay(void *pvParameters) {

}

//Connect to IoT platform adnd update the platform
void BlynkHandler(void *pvParameters) {

  while (1) {
    if (Blynk.connected()) {
      Blynk.run();
      portENTER_CRITICAL(&taskMux);
      Blynk.virtualWrite(V0, tempReading);
      Blynk.virtualWrite(V1, heartRate);
      Blynk.virtualWrite(V2, diastolic);
      Blynk.virtualWrite(V3, systolic);
      Blynk.virtualWrite(V4, stepCount);
      Blynk.virtualWrite(V5, spo2);
      portEXIT_CRITICAL(&taskMux);
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
  Serial.begin(115200);
  Serial.println("Hello World");


  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30102 was not found. Please check wiring/power."));
    while (1);
  }
  particleSensor.setup(LEDBRIGHTNESS, SAMPLEAVERAGE, LEDMODE, SAMPLE_RATE, PULSEWIDTH, ADC_RANGE);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  if (!mpu.begin())
  {
    Serial.println("Failed to initialize accelrometer!!!");
    Serial.println("Check your connections!!!");
    return;
  }
  Serial.println("MPU6050 Found! Accelerometer setup complete!");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.println("Range Set!!");

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();

  xTaskCreatePinnedToCore(countSteps, "count Steps", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(handleTemperature, "handleTemperature", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bloodPressure, "Blood Pressure", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(BlynkHandler, "Blynk Handler", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(wifiBlynkManagerTask, "WiFiBlynkManager", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // put your main code here, to run repeatedly:

}
