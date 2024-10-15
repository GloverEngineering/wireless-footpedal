// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <printf.h>
#include "Adafruit_MAX1704X.h"
//#include <Servo.h>

#define CHANNEL 1
#define LED_PIN LED_BUILTIN
#define UPDATE_BATT_DELAY 2000
#define PIN_SERVO       3
#define PIN_INPUT_SELECT  4
#define PIN_KNOB        A1
#define PIN_FOOT       A2

//max17048 maxlip
Adafruit_MAX17048 maxlipo;

// ESP32
esp_now_peer_info_t peerInfo;
uint8_t transmitterAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x22, 0x50};



//Global Variables
int led = LED_BUILTIN;
bool receivedFlag = false;
unsigned long lastUpdateMillis = 0;
int rx_input; //potentiometer raw value receieved from transmitter, 0-1023
uint16_t knob_input; //potentiometer raw value from knob, 0-1023
uint16_t foot_input; //potentiometer raw value from foot pedal, 0-1023. note it is oppositely wired from knob
uint16_t setpoint; //setpoint to send to controller via RC servo library

//Motor Controller
//Servo MotorController;

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Initialized");
  }
  else{
    Serial.println("ESP-Now Initialization Error");
    return;
  }
  //Register peer
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceived));
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  /*memcpy(peerInfo.peer_addr, transmitterAddress, 6); //arrays can't be set with =, so using memcpy to set peer_addr to receiverAddress
  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
*/

  while (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  delay(1000);
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateMillis >= UPDATE_BATT_DELAY) {
    lastUpdateMillis = currentMillis;
    CheckBattery();
    digitalWrite(LED_PIN, LOW);
  }
}


// ESP32
void onDataReceived(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&rx_input, incomingData, sizeof(rx_input));
  receivedFlag = *incomingData;
  Serial.print("Bytes received: ");
  Serial.println(data_len);
  Serial.print("data: ");
  Serial.println(rx_input);
  digitalWrite(LED_PIN, HIGH);
}

   bool ReadKnob()
  {
    double pot_input=0;
    //average 20 readings
   
    for (int i=0; i<40; i++){
      pot_input = pot_input + analogRead(PIN_KNOB);
    }
    pot_input=round(pot_input/40);

    Serial.print("Raw knob value: ");
    Serial.println(pot_input);
    knob_input=pot_input;
    return 1;
  }

  void CheckBattery()
{
  
  float cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    return;
  }
  Serial.print(F("Batt Voltage: ")); Serial.print(cellVoltage, 3); Serial.println(" V");
  Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
  Serial.println();
}
