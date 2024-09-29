  #include <Arduino.h>
  #include <Wire.h>
  #include <SPI.h>
  #include <RF24.h>
  #include <printf.h>

  #define PIN_RF_CE       10   // D9 = Chip Enable (CE) for nRF24L01+
  #define PIN_RF_CS       9  // D10 = Chip Select (CS) for nRF24L01+
  #define CHANNEL_READ    987654321
  #define CHANNEL_WRITE   123456789
  #define POT_PIN         A0


  RF24 FPRadio(PIN_RF_CE, PIN_RF_CS); //rf radio object
  bool radioReady = false; //flag

  int setpoint; //analog setpoint 0-1023
  int pot_input; //raw value
  
  void WriteData(int setpoint)
  {
    FPRadio.openWritingPipe(CHANNEL_WRITE);
    FPRadio.write(&setpoint, sizeof(setpoint));
    Serial.print("WriteData");
    Serial.println(setpoint);
    
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
  
void setup() {

  pinMode(POT_PIN, INPUT);
  Serial.begin(115200);
  printf_begin();
  Serial.println();
  Serial.println(F("RF_NANO TRANSMITTER, GLOVER ENGINEERING"));


  radioReady=FPRadio.begin();
  if (!radioReady) {
      Serial.print("Radio Initialisation Failed ");
  }
  FPRadio.setAddressWidth(5);
  FPRadio.openWritingPipe(0xF0F0F0F066LL); //address? this is what will pair two items
  FPRadio.setChannel(120);           //115 band above WIFI signals
  FPRadio.setPALevel(RF24_PA_MAX);   //MAX power low rage
  FPRadio.setDataRate(RF24_250KBPS) ;  //Maximum speed //RF24_2MBPS
  FPRadio.stopListening(); //Stop Receiving and start transminitng
  Serial.print("Send Setup Initialized");
  FPRadio.printDetails();
  delay(500);
  Serial.println(FPRadio.getDataRate());

  
  /*FPRadio.openWritingPipe(0xF0F0F0F066LL);
*/
}

void loop() {
  // put your main code here, to run repeatedly:
  ReadInput();
  
  WriteData(pot_input);
  //delay(5);
  //volts = map(volts,0,1023,0,150);
}