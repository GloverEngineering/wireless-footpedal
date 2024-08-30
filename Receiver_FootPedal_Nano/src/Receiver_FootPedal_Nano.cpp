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
  #define PIN_KNOB        A1

  enum INPUT_MODE{ //declare enumeration type
     WIRELESS,
     WIRED,
     KNOB,
  };
  
  INPUT_MODE mode; //declare enum mode of type INPUT_MODE
  int rx_input[32]; //potentiometer raw value receieved from transmitter, 0-1023
  int knob_input; //potentiometer raw value from knob, 0-1023
  int setpoint; //setpoint to send to controller via RC servo library
  
  RF24 RXRadio(PIN_RF_CE, PIN_RF_CS);
  Servo MotorController;
  
 bool ReadRx()
  {
    bool rx_connection;
    uint16_t data_size;
    if (RXRadio.available()){
      rx_connection=true;
      while(RXRadio.available()){
        data_size = RXRadio.getPayloadSize();
        RXRadio.read(rx_input, data_size);
      }
      
      Serial.print("ReadData: ");
      Serial.println(rx_input[0]);
    }
    else{
      Serial.println("no data");
      rx_connection=false;
      mode=KNOB;
      rx_input[0] = 0;
    }
    Serial.println(mode);
    return rx_connection;
  }

   bool ReadKnob()
  {
    int data;
    data=analogRead(PIN_KNOB);
    knob_input=data;
    return 1;
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
  int map_max = 180;
  int map_min = 0;
  if(!ReadRx()){ //wireless is not connected
    rx_input[0]=0; //set wireless signal to zero
  }
  ReadKnob();

  if((mode == WIRELESS)){
   
    if((knob_input>255)){ //if the knob is higher than 25%, use it to set a maximum speed - Set the max speed the foot pedal scale
      map_max = map(knob_input, 0, 1023, 0, 180);
    }
    else map_max=180;
    setpoint = map(rx_input[0], 0, 1023, 0, map_max);
  }

  if((mode == KNOB)){
    map_max=180;
    setpoint = map(knob_input, 0, 1023, 0, map_max);
  }

  Serial.print("Setpoint Value: ");
  Serial.println(setpoint);
  MotorController.write(setpoint);
  mode=WIRELESS;
  delay(20);
}