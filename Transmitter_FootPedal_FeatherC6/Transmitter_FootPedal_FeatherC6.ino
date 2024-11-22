// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX1704X.h" //power monitoring
#include <Adafruit_NeoPixel.h> //neopixel control
#include "driver/rtc_io.h"
//#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
//#include "esp_adc/adc_oneshot.h"
#include <esp_wifi.h>
#include <EEPROM.h>

//#define DEBUGSEND
#define DEBUG
//#define DEBUGPOT
#define DEBUGPAIRING
//#define DEBUGBATTERY
#define DEBUGLED
#define FWVERSION 0.2
#define POT_PIN A0
#define CHANNEL 11                 //1-14
#define UPDATE_BATT_DELAY 60000   //1 minute
#define POT_SAMPLES 64               //5hz
#define DATA_FREQ 10                //5hz
#define KEEP_ALIVE_FREQ 1         //1 hz
#define KEEP_ALIVE_TIME 5000      //5000 ms amount of time to switch from full speed transmit to keep_alive_time transmit 
#define SLEEP_TIME 60000          // milliseconds. Change to 60 seconds or longer in the future
#define PAIRING_BLINK_TIME 500    // blink fast when pairing
#define BLUE (0,255,255) 
#define LED_PWR_PIN 4
#define WIRELESS_MAX 3300
#define PAIRING_MAX 3200
#define SLEEP_ADDRESS 8 //EEPROM address to store sleep flag bit

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
uint8_t transmitterAddress[6]; // = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};

//Structures
struct foot_pedal_struct{
  int message_type; //0 data, 1 is pairing
  uint8_t battery_level; // battery level in percentage from 0-100%
  uint32_t pot_data; // only needs to be 32 bit because to average you add up 64 samples
  int setpoint_data; // if sending a mapped value from 0-180
  uint8_t MacAddress[6]; //mac address array
};

enum STATES{ //declare enumeration type
  PAIRING,
  DATA,
};

STATES mode; //variable mode of type STATES

//Global Variables
foot_pedal_struct foot_pedal_data;
foot_pedal_struct receiver_data;
int led = LED_BUILTIN;
int setpoint; //analog setpoint 0-1023
uint32_t pot_input; //raw value
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
int blinkTime = 0;
bool pairing = false;
bool led_state = false;

void pair(void){
  for(int i=0; i<sizeof(receiverAddress); i++){
    EEPROM.write(i,receiverAddress[i]);
    EEPROM.commit();
    Serial.println(EEPROM.read(i));
  }
  esp_now_unregister_recv_cb(); //unregister the callback function
  mode = DATA; //change mode back to data to get out of pairing mode
  digitalWrite(LED_PWR_PIN, HIGH); //ensure LED is on, if you switch when blinking off it stays off
  //Add peer so you don't have to power cycle
  memcpy(peerInfo.peer_addr, receiverAddress, 6); //arrays can't be set with =, so using memcpy to set peer_addr to receiverAddress
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void deletePeer(void) {
  uint8_t delStatus = esp_now_del_peer(transmitterAddress);
  if (delStatus != 0) {
    Serial.println("Could not delete peer");
  }
}

void sendData(void) {
  Serial.print("POT value: ");
  Serial.println(foot_pedal_data.pot_data);
  for (int i = 0; i < 6; i++) {
    Serial.print(receiverAddress[i]);
  }
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &foot_pedal_data, sizeof(foot_pedal_data));
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
    for (int i=0; i<POT_SAMPLES; i++){
      pot_input = pot_input + analogRead(POT_PIN);
    }
    pot_input=pot_input/POT_SAMPLES;
    foot_pedal_data.pot_data = pot_input;
    return true;
  }

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUGLED
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
  #endif
}

void onDataReceived(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  memcpy(&receiver_data, incomingData, sizeof(receiver_data));

  if (receiver_data.message_type == 1 ){ //message_type 0 is data, 1 is pairing request
    memcpy(receiverAddress, receiver_data.MacAddress, 6); //copy received mac address to global variable - now anything sent will wend correct
    for(int i=0; i<sizeof(receiverAddress); i++){
      Serial.println(receiverAddress[i]);
    }
    pair(); //write it to EEPROM, so now everytime it powers up it will read in the correct address
  }


}

void CheckBattery(){
  //WiFi.mode(WIFI_OFF);
  float cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    return;
  }
  #ifdef DEBUGBATTERY
    Serial.print(F("Batt Voltage: ")); Serial.print(cellVoltage, 3); Serial.println(" V");
    Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
  #endif

  foot_pedal_data.battery_level = maxlipo.cellPercent();
  /*
  WiFi.mode(WIFI_STA);
  esp_now_init();

  //now check with wifi on
  cellVoltage = maxlipo.cellVoltage();
  #ifdef DEBUGBATTERY
  Serial.print(F("WiFi ON Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
  #endif
  */
}

void disableInternalPower() {
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
  //adc_power_off();
  //adc_power_release()
}
void enableInternalPower() {
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
}



void setup() {
  mode = DATA;
  //serial
  Serial.begin(115200);
  Serial.println("FEATHER C6 TRANSMITTER ESP-NOW, GLOVER ENGINEERING");
  Serial.print("Firmware Version: ");
  Serial.println(FWVERSION);

  //EEPROM definition
  EEPROM.begin(9);

  //pinmodes
  pinMode(led, OUTPUT);
  pinMode(POT_PIN, INPUT);
  
  // Pairing
  //read from EEPROM unless it is 0xFF (255) regardless of what is there
  if(EEPROM.read(0)!=0xFF){
    for(int i=0; i<sizeof(receiverAddress); i++){
    receiverAddress[i] = EEPROM.read(i);
    Serial.println(receiverAddress[i]);
    }
  }

  //now check to see if we should get into a pairing state based on user input. First check EEPROM to see if we are powering up or waking up
  if(EEPROM.read(SLEEP_ADDRESS) == 0){ //it will be 0 if starting from power up. It will be 255 if waking up. Don't try pairing if waking up because you press foot pedal to wake up so we could accidentily trigger pairing
    ReadInput();
    Serial.println(pot_input);
    if(pot_input>PAIRING_MAX){ //read input immedietly upon starting up - if foot pedal is pressed fully when powering on, enter pairing mode
      mode = PAIRING;
    }
    else{
      mode = DATA;
    }
  }
  EEPROM.write(SLEEP_ADDRESS, 0); //reset it back to zero
  EEPROM.commit();

  //LED
  pinMode(LED_PWR_PIN, OUTPUT);
  digitalWrite(LED_PWR_PIN, HIGH);
  neopixel.begin();
  neopixel.clear();
  neopixel.setBrightness(15);
  //neopixel.setPixelColor(0, green));
  //neopixel.show();
  
  //wifi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //set channel
  esp_err_t result = esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  if (result == ESP_OK) {
    Serial.println("Channel Set Successfully");
  }
  else{
    Serial.println("Failed to set channel");
    return;
  }
  
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Initialized");
  }
  else{
    Serial.println("ESP-Now Initialization Error");
    return;
  }
 

  //Serial.println(esp_wifi_set_max_tx_power(8));
  WiFi.setTxPower(WIFI_POWER_19dBm); //WIFI_POWER_2dBm WIFI_POWER_19dBm
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
    delay(500);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
}

void loop() {
  unsigned long currentMillis;
  switch(mode){
    case PAIRING:
      //turn into receive mode, wait for message with mac message type of pairing, and then save MAC address to EEPROM
      esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceived));
      
      //blink LED
      currentMillis = millis();
      if((currentMillis - blinkTime) > PAIRING_BLINK_TIME) {
        digitalWrite(LED_PWR_PIN, !digitalRead(LED_PWR_PIN));
        blinkTime = millis();
      }
      #ifdef DEBUGPAIRING 
        Serial.print("paring: ");
        Serial.print(pairing);
      #endif

      break;
      // delete Serial.println(EEPROM.read(SLEEP_ADDRESS));
      
    case DATA:
      ReadInput();
      foot_pedal_data.message_type = 0;
      //pot_input >> 4;
      currentMillis = millis();
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
        if (zeroTime >= KEEP_ALIVE_TIME){ //If foot pedal has been at zero for KEEP_ALIVE_TIME e.g. 5 seconds, stop transmitting at full speed to save battery
          sendFreq = KEEP_ALIVE_FREQ;
          colorSuccess = purple;
          #ifdef DEBUGLED
            neopixel.setPixelColor(0, purple); //green
            neopixel.show();
          #endif
        }
        if (zeroTime >= (SLEEP_TIME + KEEP_ALIVE_TIME)){
          Serial.println(zeroTime);
          Serial.println("going to deep sleep now...");
          EEPROM.write(SLEEP_ADDRESS, 1); //write the sleep flag 1 immedietly when going to sleep. When waking up, set it to zero.
          EEPROM.commit();
          WiFi.mode(WIFI_OFF);
          esp_wifi_stop();
          disableInternalPower();
          //adc_power_release();
          //adc_power_off();
          delay(10);
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
          neopixel.setPixelColor(0, coral); //yellow
          neopixel.show();
          delay(100);
          neopixel.clear();
          neopixel.show();
          delay(100);
          neopixel.setPixelColor(0, coral); //yellow
          neopixel.show();
          delay(100);
          neopixel.clear();
          neopixel.show();
          delay(100);
        #endif
      }

      currentMillis = millis();
      //Serial.println((1.0/sendFreq)*1000);
      if ((currentMillis - lastUpdateMillis) >= ((1.0/sendFreq)*1000)) { //send data every sendFreq hz e.g. 5hz. use decimal for the 1 to avoid integer math
        lastUpdateMillis = currentMillis;
        sendData();
      }

      break;
   
  }
}
