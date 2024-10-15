#include "WiFi.h"
//#include <ESP8266WiFi.h> // for ESP8266 boards

void setup(){
    Serial.begin(115200);
    delay(5000); // uncomment if your serial monitor is empty
    WiFi.mode(WIFI_STA); 
    while (!(WiFi.STA.started())) { // comment the while loop for ESP8266
        delay(10);
    }
    // delay(1000); // uncomment for ESP8266
    Serial.print("MAC-Address: ");
    delay(2000);
    Serial.print("MAC-Address: ");
    String mac = WiFi.macAddress();
    Serial.println(mac);
    
    Serial.print("Formated: ");
    Serial.print("{");
    int index = 0;
    for(int i=0; i<6; i++){
        Serial.print("0x");
        Serial.print(mac.substring(index, index+2));
        if(i<5){
            Serial.print(", ");
        }
        index += 3;
    }
    Serial.println("}");
}
 
void loop(){}
