//
// Created by Roman on 12.12.2019.
//

#include <Arduino.h>
#include "PinsTTGO.h"
#include "LoRa.h"

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
        while (true);                   // if failed, do nothing
    }

    // Sync Words will not receive each other's transmissions. This is one way you can filter out radios you want to ignore, without making an addressing scheme.
    //LoRa.setSyncWord(0xF3);           // ranges from 0-0xFF, default 0x34, see API docs

    // Spreading factor affects reliability of transmission at high rates, however, avoid a large spreading factor when you're sending continually.
    //LoRa.setSpreadingFactor(8);           // ranges from 6-12,default 7 see API docs
}

void print_wakeup_reason(){
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
        case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
        case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
        case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
        default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }
}

void transmitMeasurement() {
    Serial.print("Sending packet: ");

    LoRa.beginPacket();
    LoRa.print(globalCounter);
    LoRa.print("/20.0C");
    LoRa.print("/1013hPa");
    LoRa.print("/50rH");
    LoRa.endPacket();
    globalCounter++;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    print_wakeup_reason();
    initLoRa();
    transmitMeasurement();

    Serial.println("Going to sleep.");
    //esp_sleep_enable_timer_wakeup(5000000);
    //esp_deep_sleep_start();
    esp_deep_sleep(SLEEP_IN_US);
}

void loop() {

}