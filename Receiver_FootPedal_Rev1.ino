  #include <Arduino.h>
  #include <Wire.h>
  #include <SPI.h>
  #include <RF24.h>
  #include <printf.h>
  #include <Servo.h>
  
  #define PIN_RF_CE       10   // D9 = Chip Enable (CE) for nRF24L01+
  #define PIN_RF_CS       9  // D10 = Chip Select (CS) for nRF24L01+
  #define CHANNEL_READ    123456789
  #define CHANNEL_WRITE   987654321
  #define PIN_SERVO       2

  enum INPUT_MODE{
     WIRELESS,
     WIRED,
     KNOB,
  };
  
  INPUT_MODE mode;
  int potentiometer[32];
  int setpoint;
  
  RF24 RXRadio(PIN_RF_CE, PIN_RF_CS);
  Servo MotorController;
  
  void ReadData()
  {
    uint16_t data_size;
    if (RXRadio.available()){
      while(RXRadio.available()){
        data_size = RXRadio.getPayloadSize();
        RXRadio.read(potentiometer, data_size);
      }
      setpoint = map(potentiometer[0], 0, 1023, 0, 180);
      MotorController.write(setpoint);
      mode=WIRELESS;
      
      Serial.print("ReadData: ");
      Serial.println(potentiometer[0]);
      Serial.print("Setpoint Value: ");
      Serial.println(setpoint);
    }
    else{
      Serial.println("no data");
      mode=KNOB;
    }
    Serial.println(mode);
  }
  
 
  
void setup() {

  Serial.begin(115200);
  printf_begin();
  Serial.println();
  Serial.println(F("RF_NANO v3.0 Recieve Test"));

  //SelfTest;

  MotorController.attach(PIN_SERVO);

  RXRadio.begin();
  RXRadio.setAddressWidth(5);
  RXRadio.openReadingPipe(1, CHANNEL_READ);
  RXRadio.setChannel(115);           //115 band above WIFI signals
  RXRadio.setPALevel(RF24_PA_MIN);   //MIN power low rage
  RXRadio.setDataRate(RF24_2MBPS) ;  //Minimum speed
  RXRadio.startListening(); //Stop Receiving and start transminitng
  Serial.print("Receive Setup Initialized");
  RXRadio.printDetails();
  delay(500);
    
}

void loop() {
  // put your main code here, to run repeatedly:
  ReadData();
  delay(20);
}
