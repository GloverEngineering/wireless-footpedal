// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX1704X.h" //power monitoring
#include <Adafruit_NeoPixel.h> //neopixel control
#include "driver/rtc_io.h"

//#define DEBUGSEND
//#define DEBUG
//#define DEBUGPOT
#define DEBUGBATTERY
#define POT_PIN A0
#define CHANNEL 1                 //1-14
#define UPDATE_BATT_DELAY 60000   //1 minute
#define POT_FREQ 64               //5hz
#define DATA_FREQ 10                //5hz
#define KEEP_ALIVE_FREQ 1         //1 hz
#define KEEP_ALIVE_TIME 5000      //5000 ms
#define SLEEP_TIME 10000          // seconds. Change to 60 seconds or longer in the future
#define BLUE (0,255,255) 
//NEOPIXEL_I2C_POWER IO20
//PIN_NEOPIXEL IO9
//LED_BULTIN 15
//sleep stuff
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define WAKEUP_GPIO              GPIO_NUM_1     // Only RTC IO are allowed - ESP32 Pin example


//Objects
Adafruit_MAX17048 maxlipo; //max17048 maxlip object
Adafruit_NeoPixel neopixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

//LED colors
uint32_t blue = neopixel.Color(0, 0, 255);
uint32_t green = neopixel.Color(0, 150, 0);
uint32_t yellow = neopixel.Color(153, 153, 0);
uint32_t coral = neopixel.Color(255, 127, 80);
uint32_t red = neopixel.Color(139, 0, 0);
uint32_t purple = neopixel.Color(128, 0, 128);
uint32_t colorFail = red; //innitialize a blank color variable
uint32_t colorSuccess = green; //innitialize a blank color variable


// ESP32
esp_now_peer_info_t peerInfo;
uint8_t transmitterAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x22, 0x50};


//Global Variables
int led = LED_BUILTIN;
int setpoint; //analog setpoint 0-1023
long pot_input; //raw value
unsigned long lastUpdateMillis = 0;
unsigned long lastUpdateBattMillis = 0;
unsigned long lastUpdateSleep = 0;
uint8_t flagToSend = 0;
uint8_t send_failure_count = 0;
uint8_t rx_failure_count = 0;
unsigned long sendFreq = DATA_FREQ;
bool zeroFlag = false;
int zeroTime = 0;
int zeroStart = 0;

void setup() {
  
//serial and pinmodes
  Serial.begin(115200);
  Serial.println("FEATHER C6 TRANSMITTER ESP-NOW, GLOVER ENGINEERING");
  pinMode(led, OUTPUT);
  pinMode(POT_PIN, INPUT);

//LED
  neopixel.begin();
  neopixel.clear();
  neopixel.setBrightness(15);
  //neopixel.setPixelColor(0, green));
  //neopixel.show();
  
//wifi
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
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
}

void deletePeer(void) {
  uint8_t delStatus = esp_now_del_peer(transmitterAddress);
  if (delStatus != 0) {
    Serial.println("Could not delete peer");
  }
}

void sendData(void) {
  Serial.print("POT value: ");
  Serial.println(pot_input);
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &pot_input, sizeof(pot_input));
  //the error codes are mostly for issues with peers and stuff
  #ifdef DEBUGSEND
  if (result == ESP_OK) {
    Serial.println("Send Success");
    neopixel.setPixelColor(0, blue); //blue
    neopixel.show();
    //Serial.println(WiFi.getTxPower());
  } else{
    Serial.print("Send Error failure count: ");
    neopixel.setPixelColor(0, coral); //orange coral
    neopixel.show();
    send_failure_count++;
    Serial.println(send_failure_count);
  }
  #endif
}

int ReadInput()
  {
    pot_input = 0;
    for (int i=0; i<POT_FREQ; i++){
      pot_input = pot_input + analogRead(POT_PIN);
    }
    pot_input=pot_input/POT_FREQ;
    
    return true;
  }

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  
  if(status == !ESP_NOW_SEND_SUCCESS){
    rx_failure_count++;
    neopixel.setPixelColor(0, colorFail); //red
    neopixel.show();
  }
  if(status == ESP_NOW_SEND_SUCCESS){
    neopixel.setPixelColor(0, colorSuccess); //green
    neopixel.show();
  }
  Serial.print("rx_failure_count: ");
  Serial.println(rx_failure_count);

}

void CheckBattery(){
  float cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    return;
  }
  #ifdef DEBUGBATTERY
  Serial.print(F("Batt Voltage: ")); Serial.print(cellVoltage, 3); Serial.println(" V");
  Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
  #endif
}

void disableInternalPower() {
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
}
void enableInternalPower() {
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
}
void loop() {
  
  ReadInput();
  //pot_input >> 4;
  unsigned long currentMillis = millis();
  #ifdef DEBUGPOT
  Serial.print("POT value: ");
  Serial.println(pot_input);
  #endif
  //Serial.print("Time to read: ");
  //Serial.println(millis()-currentMillis);

  if (pot_input <= 2){
    if (zeroFlag == false){ //if this is the first time pot_input is zero, set a flag and start counting
      zeroStart = millis();
      zeroFlag = true; //trigger once, set it to true
    }
    zeroTime = millis() - zeroStart;
    if (zeroTime >= KEEP_ALIVE_TIME){ //If foot pedal has been at zero for PAUSE_TIME e.g. 5 seconds, stop transmitting at full speed to save battery
      sendFreq = KEEP_ALIVE_FREQ;
      colorSuccess = purple;
      neopixel.setPixelColor(0, purple); //green
      neopixel.show();
    }
    if (zeroTime >= (SLEEP_TIME + KEEP_ALIVE_TIME)){
      Serial.println(zeroTime);
      Serial.println("going to deep sleep now...");
      disableInternalPower();
      esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_deep_sleep_start();
    } 
  }  else{
    zeroFlag = false; //pot_input went positive so reset the flat
    sendFreq = DATA_FREQ;
    colorSuccess = green;
  }
  
  currentMillis = millis();
  if (currentMillis - lastUpdateBattMillis >= UPDATE_BATT_DELAY) { //check battery every e.g. 60 seconds
    lastUpdateBattMillis = currentMillis;
    CheckBattery();
    #ifdef DEBUGBATTERY
    neopixel.setPixelColor(0, yellow); //yellow
    neopixel.show();
    delay(250);
    neopixel.clear();
    neopixel.show();
    delay(250);
    neopixel.setPixelColor(0, yellow); //yellow
    neopixel.show();
    delay(250);
    neopixel.clear();
    neopixel.show();
    delay(250);
    #endif
  }

  currentMillis = millis();
  //Serial.println((1.0/sendFreq)*1000);
  if ((currentMillis - lastUpdateMillis) >= ((1.0/sendFreq)*1000)) { //send data every sendFreq hz e.g. 5hz. use decimal for the 1 to avoid integer math
    lastUpdateMillis = currentMillis;
    sendData();
  }
/*
  if (currentMillis - lastUpdateSleep >= SLEEP_TIME){
      esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
      Serial.println("going to deep sleep now...");
      esp_deep_sleep_start();
  }
  */
  
  //delay(40); //run at 10hz
}
