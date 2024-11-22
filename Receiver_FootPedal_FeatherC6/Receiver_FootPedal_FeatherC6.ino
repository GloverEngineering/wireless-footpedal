// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <printf.h> 
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h> //neopixel control
#include <ESP32Servo.h>
#include <esp_wifi.h>

//#define DEBUGSEND
#define DEBUGWIRELESS
//#define DEBUGPOT
//#define DEBUGPRINT
//#define DEBUGBATTERY
#define FWVERSION 0.2
#define CHANNEL 11
#define LED_PIN LED_BUILTIN
#define UPDATE_BATT_DELAY 60000   //once per minute: time in milliseconds
#define POT_SAMPLES 64            //5hz
#define DATA_FREQ 10              //hz
#define KEEP_ALIVE_FREQ 1         //1 hz
#define PIN_SERVO 4 //6
#define PIN_KNOB  A0
#define PIN_FOOT  A2
#define KNOB_MAX 3220
#define FOOT_MAX 3311
#define WIRELESS_MAX 3350
//NEOPIXEL_I2C_POWER IO20
//PIN_NEOPIXEL IO9
//LED_BULTIN 15

//Objects
Adafruit_MAX17048 maxlipo; //max17048 maxlip object
Adafruit_NeoPixel neopixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); //LED NEOpixel object
Servo MotorController; //servo class object

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
uint8_t BroadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t transmitterAddress[6]; // = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[6]; // = {0x9C, 0x9E, 0x6E, 0x5B, 0x6C, 0xE8};

//Structures
struct foot_pedal_struct{
  int message_type; //0 data, 1 is pairing
  uint8_t battery_level; // battery level in percentage from 0-100%
  uint32_t pot_data; // only needs to be 32 bit because to average you add up 64 samples
  int setpoint_data; // if sending a mapped value from 0-180
  uint8_t MacAddress[6]; //mac address array
};

//Global Variables
foot_pedal_struct foot_pedal_data;
foot_pedal_struct outgoing_data;
int led = LED_BUILTIN;
bool receivedFlag = false;
unsigned long lastUpdateMillis = 0;
unsigned long lastReceived = 0;
uint32_t rx_input; //potentiometer raw value receieved from transmitter, 0-1023
uint16_t knob_input; //potentiometer raw value from knob, 0-1023
uint16_t foot_input; //potentiometer raw value from foot pedal, 0-1023. note it is oppositely wired from knob
uint16_t setpoint; //setpoint to send to controller via RC servo library

enum INPUT_MODE{ //declare enumeration type
     WIRELESS,
     WIRED,
     KNOB,
  };
  
INPUT_MODE mode; //declare enum mode of type INPUT_MODE

// ESP32
void onDataReceived(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  lastReceived = millis();

  memcpy(&foot_pedal_data, incomingData, sizeof(foot_pedal_data));
  rx_input = foot_pedal_data.pot_data;
  receivedFlag = *incomingData;
  #ifdef DEBUGWIRELESS
    Serial.print("Bytes received: ");
    Serial.println(data_len);
    Serial.print("Averaged wireless foot pedal value: ");
    Serial.println(rx_input);
  #endif

  neopixel.setPixelColor(0, green); //green
  neopixel.show();
  mode = WIRELESS;

}

bool ReadFootpedal()
  {
    double pot_input=0;
    //average POT_SAMPLES # of readings
    for (int i=0; i<POT_SAMPLES; i++){
      pot_input = pot_input + analogRead(PIN_FOOT);
    }
    pot_input=round(pot_input/POT_SAMPLES);

    Serial.print("Averaged wired foot pedal value: ");
    Serial.println(pot_input);
    foot_input=pot_input;
    return 1;
  }

bool ReadKnob(){
    double pot_input=0;
    //average POT_SAMPLES # of readings
    for (int i=0; i<POT_SAMPLES; i++){
      pot_input = pot_input + analogRead(PIN_KNOB);
    }
    pot_input=round(pot_input/POT_SAMPLES);

    Serial.print("Averaged knob value: ");
    Serial.println(pot_input);
    knob_input=pot_input;
    return 1;
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


void setup() {
  Serial.begin(115200);
  Serial.println("FEATHER C6 TRANSMITTER ESP-NOW, GLOVER ENGINEERING");
  Serial.print("Firmware Version: ");
  Serial.println(FWVERSION);
//pins
  pinMode(PIN_FOOT, INPUT);
  pinMode(PIN_KNOB, INPUT);
  pinMode(led, OUTPUT);
//Motor Controller
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  MotorController.setPeriodHertz(50);
  MotorController.attach(PIN_SERVO, 1000, 2000);
  
//LED
  neopixel.begin();
  neopixel.clear();
  neopixel.setBrightness(15);

  //WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
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

  //Register call back function for receiving data
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceived));

  //Send mac address once every power up to pair with a foot pedal that is in pairing mode
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  WiFi.setTxPower(WIFI_POWER_19dBm);
  memcpy(peerInfo.peer_addr, BroadcastAddress, 6); //arrays can't be set with =, so using memcpy to set peer_addr to receiverAddress
  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //get board mac address and convert to byte array. Thanks chatgpt
  String macStr = WiFi.macAddress();
  for (int i = 0; i < 6; i++) {
    outgoing_data.MacAddress[i] = (uint8_t)strtol(macStr.substring(i * 3, i * 3 + 2).c_str(), NULL, 16);
    Serial.print(outgoing_data.MacAddress[i]);
  }
  outgoing_data.message_type = 1;
  esp_err_t result2 = esp_now_send(BroadcastAddress, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  if (result2 == ESP_OK) { Serial.println("Send Success");
    } else{Serial.print("Send Error");}

  //now delete the broadcast peer
  esp_now_del_peer(BroadcastAddress);
  

}

void loop() {
  int map_max = 180;
  int map_min = 0;
  static int last_read = 0; //variable to store the last updated value

  //the ondatareceieved routin will run as an interrupt as soon as any wireless data is received. When it does we set the mode to WIRELESS
  //So we have to check for if the connection is lost and set the mode accordingly

//Check if foot pedal is no longer connected
  unsigned long currentMillis = millis();
  if (currentMillis - lastReceived >= (1.0/KEEP_ALIVE_FREQ)*1000){ //if we haven't received data for 2x the light-sleep datarate (e.g. if data is sent at 1hz when sleeping, then we receive every 1000ms and if we don't for 2000ms then we have lost connection)
    neopixel.clear();
    neopixel.show();
    mode = WIRED; //Wireless no longer connected
  }

//Look at what mode we are in to set the setpoint. If its wireless, use wireless, otherwise use whichever is larger from the knob or wired pedal
  if(mode == WIRED){
    ReadKnob(); //only read the inputs if wireless is not connected
    ReadFootpedal();
    if(knob_input>foot_input){ //use whichever input is larger
       map_max=180;
      setpoint = map(knob_input, 0, KNOB_MAX, 0, map_max);
    }
    else{
      map_max=180;
      setpoint = map(foot_input, 0, FOOT_MAX, 0, map_max);
    }
  }
  else{ //wireless mode
    //Moved the map function to the foot pedal. So the foot pedal transmits the 0-180 value
    map_max=180;
    setpoint = map(rx_input, 0, FOOT_MAX, 0, map_max);
  }

  Serial.print("Mode: ");
  Serial.println(mode);
  Serial.print("Setpoint Value: ");
  Serial.println(setpoint);

  MotorController.write(setpoint);


}
