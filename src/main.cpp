//
// ESP32 in DeepSleep reading a BME280 every few seconds
//

#include <Arduino.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_BME280.h>
#include "DataStructures.h"
#include "PinsTTGO.h"

#define SERIAL_BAUD 9600
//433000000 for Asia
//866000000 for Europe
//915000000 for North America
#define LORA_BAND 866000000
#define SLEEP_IN_US 5000000

RTC_DATA_ATTR long globalCounter=1;

void initLoRa() {
    LoRa.setPins(TTGO_PIN_NSS, TTGO_PIN_RST, TTGO_PIN_DIO0); // set CS, reset, IRQ pin

    if (!LoRa.begin(LORA_BAND)) {         // initialize ratio at 915 MHz
        Serial.println("LoRa init failed. Check your connections.");
        while(true);                   // if failed, do nothing
        while(true);                   // if failed, do nothing
    }
    //LoRa.setSyncWord(0xF3);           // ranges from 0-0xFF, default 0x34, see API docs
    //LoRa.setSpreadingFactor(8);           // ranges from 6-12,default 7 see API docs
}

void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0 :
            Serial.println("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1 :
            Serial.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER :
            Serial.println("Wakeup caused by timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD :
            Serial.println("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP :
            Serial.println("Wakeup caused by ULP program");
            break;
        default :
            Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
            break;
    }
}

weatherData readSensor(uint8_t bmeAddress = 0x76) {
    Adafruit_BME280 bme; // I2C

    if (!bme.begin(bmeAddress)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while(true);
    }

    weatherData measurement;
    measurement.temperature = bme.readTemperature();
    measurement.pressure = bme.readPressure() / 100.0F;
    measurement.humidity = bme.readHumidity();

    return measurement;
}

void printValues(weatherData dataToPrint) {
    Serial.print("Temperature = ");
    Serial.print(dataToPrint.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(dataToPrint.pressure);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(dataToPrint.humidity);
    Serial.println(" %");

    Serial.println();
}

void transmitMeasurement(weatherData dataToPrint) {
    Serial.print("Sending LoRa packet...");

    LoRa.beginPacket();
    LoRa.print(globalCounter);
    LoRa.print(0x20);
    LoRa.print(dataToPrint.temperature);
    LoRa.print(0x20);
    LoRa.print(dataToPrint.pressure);
    LoRa.print(0x20);
    LoRa.print(dataToPrint.humidity);
    LoRa.endPacket();
    globalCounter++;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    print_wakeup_reason();

    const weatherData &newValues = readSensor();
    printValues(newValues);

    initLoRa();
    transmitMeasurement(newValues);

    Serial.println("Going to sleep.");
    esp_deep_sleep(SLEEP_IN_US);
}

void loop() {
    // this is never reached
}