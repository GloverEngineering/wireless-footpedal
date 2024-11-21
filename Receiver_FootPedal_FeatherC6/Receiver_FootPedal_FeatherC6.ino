test
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
#define DEBUGBATTERY
#define FWVERSION 0.1
#define CHANNEL 11
#define LED_PIN LED_BUILTIN
#define UPDATE_BATT_DELAY 60000   //once per minute: time in milliseconds
#define POT_SAMPLES 64            //5hz
#define DATA_FREQ 10              //hz
#define KEEP_ALIVE_FREQ 1         //1 hz
#define PIN_SERVO 4 //6
//#define PIN_INPUT_SELECT  4
#define PIN_KNOB  A0
#define PIN_FOOT  A2
#define KNOB_MAX 3220
#define FOOT_MAX 3311
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
uint8_t transmitterAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0x9C, 0x9E, 0x6E, 0x5B, 0x6C, 0xE8};

//Global Variables
int led = LED_BUILTIN;
bool receivedFlag = false;
unsigned long lastUpdateMillis = 0;
unsigned long lastReceived = 0;
uint16_t rx_input; //potentiometer raw value receieved from transmitter, 0-1023
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

  memcpy(&rx_input, incomingData, sizeof(rx_input));
  receivedFlag = *incomingData;
  #ifdef DEBUGWIRELESS
    Serial.print("Bytes received: ");
    Serial.println(data_len);
  #endif
  Serial.print("Averaged wireless foot pedal value: ");
  Serial.println(rx_input);
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

    //set channel
  /*
  esp_err_t result = esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  if (result == ESP_OK) {
    Serial.println("Channel Set Successfully");
  }
  else{
    Serial.println("Failed to set channel");
    return;
  }
*/
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
    delay(500);
  }

  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);
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

/* only used for development
  currentMillis = millis();
  if (currentMillis - lastUpdateMillis >= UPDATE_BATT_DELAY) { //every UPDATE_BATT_DELAY seconds, check the battery
    lastUpdateMillis = currentMillis;
    CheckBattery();
  }
*/
  Serial.print("Mode: ");
  Serial.println(mode);
  Serial.print("Setpoint Value: ");
  Serial.println(setpoint);

  MotorController.write(setpoint);


}
