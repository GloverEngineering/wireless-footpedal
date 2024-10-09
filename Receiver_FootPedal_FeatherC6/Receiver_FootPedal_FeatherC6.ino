// ESP32 Imports
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 14
#define LED_PIN LED_BUILTIN
#define UPDATE_DELAY 50

// ESP32
esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x21, 0x38};
uint8_t receiverAddress[] = {0xF0, 0xF5, 0xBD, 0x31, 0x22, 0x50};

bool receivedFlag = false;
unsigned long lastUpdateMillis = 0;

//Global Variables
int led = LED_BUILTIN;


void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  esp_now_init();
  peerInfo.channel = CHANNEL;
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceived));
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdateMillis >= UPDATE_DELAY) {
    digitalWrite(LED_PIN, receivedFlag);
    lastUpdateMillis = currentMillis;
  }
}


// ESP32
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  receivedFlag = *data;
  Serial.print("Bytes received: ");
  Serial.println(data_len);
}
