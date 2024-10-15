// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX1704X.h"

#define POT_PIN         A0

#define CHANNEL 1 //1-14
#define UPDATE_BATT_DELAY 2000

//max17048 maxlip
Adafruit_MAX17048 maxlipo;


// ESP32
esp_now_peer_info_t peerInfo;
uint8_t transmitterAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x22, 0x50};



//Global Variables
int led = LED_BUILTIN;
int setpoint; //analog setpoint 0-1023
int pot_input; //raw value
unsigned long lastUpdateMillis = 0;
uint8_t flagToSend = 0;
uint8_t send_failure_count = 0;
uint8_t rx_failure_count = 0;

void setup() {
  
  Serial.begin(115200);
  Serial.println("FEATHER C6 TRANSMITTER ESP-NOW, GLOVER ENGINEERING");
  pinMode(led, OUTPUT);
  pinMode(POT_PIN, INPUT);
  

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Initialized");
  }
  else{
    Serial.println("ESP-Now Initialization Error");
    return;
  }

  //Serial.println(esp_wifi_set_max_tx_power(8));
  WiFi.setTxPower(WIFI_POWER_15dBm); //WIFI_POWER_2dBm 
  Serial.println(WiFi.getTxPower());
  delay(5000);

  //Register peer
  esp_now_register_send_cb(OnDataSent); //call back function
  peerInfo.channel = CHANNEL;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, receiverAddress, 6); //arrays can't be set with =, so using memcpy to set peer_addr to receiverAddress
  
  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  

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
 
  ReadInput();
  Serial.println(pot_input);

  
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateMillis >= UPDATE_BATT_DELAY) {
//    if (esp_now_is_peer_exist(transmitterAddress)) {
//      sendData();
//      Serial.println("sending");
//    }
    lastUpdateMillis = currentMillis;
    CheckBattery();
  }
  sendData();
  
  delay(40); //run at 10hz
}



void deletePeer(void) {
  uint8_t delStatus = esp_now_del_peer(transmitterAddress);
  if (delStatus != 0) {
    Serial.println("Could not delete peer");
  }
}

void sendData(void) {
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &pot_input, sizeof(pot_input));
  if (result == ESP_OK) {
    Serial.println("Send Success");
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    Serial.println(WiFi.getTxPower());
  } else{
    Serial.print("Send Error failure count: ");
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    send_failure_count++;
    Serial.println(send_failure_count);
  }
}

int ReadInput()
  {
    pot_input = 0;
    for (int i=0; i<20; i++){
      pot_input = pot_input + analogRead(POT_PIN);
    }
    pot_input=pot_input/20;
    
    return true;
  }

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if(status == !ESP_NOW_SEND_SUCCESS){
    rx_failure_count++;
  }
  Serial.println(rx_failure_count);

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
  Serial.print("Receive failure count is");
  Serial.println(rx_failure_count);
}
