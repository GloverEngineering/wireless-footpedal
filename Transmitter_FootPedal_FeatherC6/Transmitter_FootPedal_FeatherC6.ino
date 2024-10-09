// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL 14 //1-14
#define UPDATE_DELAY 250

// ESP32
esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x22, 0x50};

unsigned long lastUpdateMillis = 0;
uint8_t flagToSend = 0;

//Global Variables
int led = LED_BUILTIN;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led, OUTPUT);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  peerInfo.channel = CHANNEL;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, receiverAddress, 6); //arrays can't be wet with =, so using memcpy
  
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello!");
  
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                // wait for a half second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(500); 
  
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateMillis >= UPDATE_DELAY) {
    flagToSend = !flagToSend; // Invert boolean value
    if (esp_now_is_peer_exist(broadcastAddress)) {
      sendData();
    }
    lastUpdateMillis = currentMillis;
  }
}



void deletePeer(void) {
  uint8_t delStatus = esp_now_del_peer(broadcastAddress);
  if (delStatus != 0) {
    Serial.println("Could not delete peer");
  }
}

void sendData(void) {
  uint8_t result = esp_now_send(broadcastAddress, &flagToSend, sizeof(flagToSend));
  if (result != 0) {
    Serial.print("Error sending data!");
    deletePeer();
  }
}
