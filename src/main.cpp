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

// for now: measure and transmit every 5 minutes
#define uS_TO_S_FACTOR 1000000
#define SLEEP_SECONDS 300

/*
 * Very basic LoRa initialization.
 * syncWord and spreadingFactor are currently unused but might be interesting parameters to tune/optimize your setup.
 */
void initLoRa() {
    LoRa.setPins(TTGO_PIN_NSS, TTGO_PIN_RST, TTGO_PIN_DIO0); // set CS, reset, IRQ pin

    if (!LoRa.begin(LORA_BAND)) {         // initialize LoRa ratio at LORA_BAND
        Serial.println("LoRa init failed. Check your connections.");
        while (true);                   // if failed, do nothing
    }

//    LoRa.setTxPower(12);                // explicitly set higher Tx power if needed
//    LoRa.setSyncWord(0xF3);             // ranges from 0-0xFF, default 0x34, see API docs
//    LoRa.setSpreadingFactor(8);         // ranges from 6-12,default 7 see API docs
}

/*
 * Currently only for testing - print the wakeup reason to serial.
 * Can be used to handle special wakeup causes depending on your application.
 */
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

/*
 * Performs sensor initialization and performs a single measurement. We do not use the global sensor-variable because of sleep mode.
 */
weatherData readSensor(uint8_t bmeAddress = 0x76) {
    Adafruit_BME280 bme; // I2C

    if (!bme.begin(bmeAddress)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (true);
    }

    weatherData measurement;
    measurement.temperature = bme.readTemperature();
    measurement.pressure = bme.readPressure() / 100.0F;
    measurement.humidity = bme.readHumidity();

    return measurement;
}

/*
 * Prints measurements to serial
 */
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

/*
 * Transmits our data as a "well readable" LoRa-Packet.
 */
void transmitMeasurement(weatherData dataToPrint) {
    Serial.print("Sending LoRa packet...");

    /*
    * Floats of our measurements with 1/2 digits after separator should be perfect, higher accuracy of the chip is not possible
    * The separators are not needed for LoRa. I added them here for better readability if testing with a simple LoRa receiver.
    */
    LoRa.beginPacket();
//    LoRa.print(globalCounter);
//    LoRa.print(',');
    LoRa.print(dataToPrint.temperature, 1);
    LoRa.print(',');
    LoRa.print(dataToPrint.pressure, 2);
    LoRa.print(',');
    LoRa.print(dataToPrint.humidity, 1);
    LoRa.endPacket();

}

/**
 * there is no official hibernate-function yet for ESP32 but according to the documentation this should have the same effect.
 */
void hibernateMode(uint64_t time_in_us) {
    esp_sleep_enable_timer_wakeup(time_in_us);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
}

/*
 * Typical Arduino setup-function. This is where our main code runs! After a wakeup we run into setup, execute our code and go to sleep again.
 */
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    print_wakeup_reason();

    const weatherData &newValues = readSensor();
    printValues(newValues);

    initLoRa();

    //print values (for testing/debugging)
    printValues(newValues);

    //send values over LoRa
    transmitMeasurement(newValues);

    // this calls a LoRa.sleep() and powers off the SPI pins - should save some power
    LoRa.end();

    Serial.println("Going to sleep.");
    hibernateMode(SLEEP_SECONDS * uS_TO_S_FACTOR);
}

/*
 * Typical Arduino loop-function. This is never reached as after a wakeup we run into setup,execute our code and go to sleep again.
 */
void loop() {
    // this is never reached
}