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


  RF24 FPRadio(PIN_RF_CE, PIN_RF_CS);
  bool radioReady = false;

  int setpoint;
  double pot;
  
  void WriteData(int setpoint)
  {
    FPRadio.openWritingPipe(CHANNEL_WRITE);
    FPRadio.write(&setpoint, sizeof(setpoint));
    Serial.print("WriteData");
    Serial.println(setpoint);
    
  }
  void SelfTest()
  {
    FPRadio.begin();
    FPRadio.setAddressWidth(5);
    FPRadio.openReadingPipe(0, 0x1212121212LL);
    FPRadio.openReadingPipe(1, 0x3434343431LL);
    FPRadio.openReadingPipe(2, 0x3434343432LL);
    FPRadio.openReadingPipe(3, 0x3434343433LL);
    FPRadio.openReadingPipe(4, 0x3434343434LL);
    FPRadio.openReadingPipe(5, 0x3434343435LL);
    FPRadio.setChannel(115);            //115 band above WIFI signals
    FPRadio.setPALevel(RF24_PA_MAX);    //MIN power low rage
    FPRadio.setDataRate(RF24_1MBPS) ;   //Minimum speed
    Serial.println("Setup Initialized");
    FPRadio.printDetails();
    
  }
  
void setup() {

  pinMode(POT_PIN, INPUT);
  Serial.begin(115200);
  printf_begin();
  Serial.println();
  Serial.println(F("RF_NANO v3.0 Test"));

  //SelfTest;

  radioReady=FPRadio.begin();
  if (!radioReady) {
      Serial.print("Radio Initialisation Failed ");
  }
  FPRadio.setAddressWidth(5);
  FPRadio.openWritingPipe(0xF0F0F0F066LL);
  FPRadio.setChannel(115);           //115 band above WIFI signals
  FPRadio.setPALevel(RF24_PA_MIN);   //MIN power low rage
  FPRadio.setDataRate(RF24_2MBPS) ;  //Minimum speed
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
  pot = 0;
  for (int i=0; i<20; i++){
    pot = pot + analogRead(POT_PIN);
  }
  pot=pot/20;
  
  WriteData(pot);
  //delay(5);
  //volts = map(volts,0,1023,0,150);
}
