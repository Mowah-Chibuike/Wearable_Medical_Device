#ifndef _MEDIC_DEV_H
#define _MEDIC_DEV_H

#define BLYNK_TEMPLATE_ID "TMPL2qXw9ZLGR"
#define BLYNK_TEMPLATE_NAME "Wearable Medical Device"
#define BLYNK_AUTH_TOKEN "3cwZJDgxPtYjE-A8qdCcD2I9iRhYl0Ky"


#include <Adafruit_MLX90614.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define MAX_BRIGHTNESS 255
#define ADC_RANGE 4096
#define SAMPLE_RATE 100
#define PULSEWIDTH 411
#define LEDMODE 2
#define SAMPLEAVERAGE 4
#define LEDBRIGHTNESS 60

#define BUFFER_LENGTH 100


uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

char ssid[] = "Timothy's A16";
char pass[] = "nmg9h28vmhjth6m";

#endif /* _MEDIC_DEV_H */