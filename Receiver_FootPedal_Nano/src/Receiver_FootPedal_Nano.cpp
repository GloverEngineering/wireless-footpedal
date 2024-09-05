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
  #define PIN_INPUT_SELECT  3
  #define PIN_KNOB        A1
  #define PIN_PEDAL       A2
  #define MAP_MAX_ROBOCLOAW 1910

  enum INPUT_MODE{ //declare enumeration type
     WIRELESS,
     WIRED,
     KNOB,
  };
  
  INPUT_MODE mode; //declare enum mode of type INPUT_MODE
  int rx_input[32]; //potentiometer raw value receieved from transmitter, 0-1023
  uint16_t knob_input; //potentiometer raw value from knob, 0-1023
  uint16_t setpoint; //setpoint to send to controller via RC servo library
  
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
    else{ //no rx data, use the knob input
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
    double pot_input=0;
    //average 20 readings
   
    for (int i=0; i<40; i++){
      pot_input = pot_input + analogRead(PIN_KNOB);
    }
    pot_input=round(pot_input/40);

    Serial.print("Raw knob value: ");
    Serial.println(pot_input);
    knob_input=pot_input;
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
  int map_max = MAP_MAX_ROBOCLOAW;
  int map_min = 1500;
  static int last_read = 0; //variable to store the last updated value

  if(!ReadRx()){ //ReadRX and if wireless is not connected set its output to zero as a fail safe
    rx_input[0]=0; //set wireless signal to zero
  }
  ReadKnob();
  //Filter the knob input beyohnd the hardware RC filter. RC filter f_c is set at about 2 hz
  /*
  if(abs(knob_input-last_read)>2){ //If the pot has changed by more than 4 counts, write the data. Otherwise keep the previous data
    last_read = knob_input; //update the last read to the new value
  }
  else knob_input=last_read; //knob didn't move enough, set it to the last good value
*/
  if((mode == WIRELESS)){
   
    if((knob_input>255)){ //if the knob is higher than 25%, use it to set a maximum speed - Set the max speed the foot pedal scale
      map_max = map(knob_input, 0, 1023, map_min, MAP_MAX_ROBOCLOAW);
    }
    else map_max=MAP_MAX_ROBOCLOAW; //else if the knob is less than 25% let the max value be the max value

    setpoint = map(rx_input[0], 0, 1023, map_min, map_max); //set the setpoint based on the scaled wireless input. rx_input is filtered on the footpedal before being sent
  }

  if((mode == KNOB)){ //no transmit is detected i.e. no wireless foot pedal and set the setpoint based on the knob input
    map_max=MAP_MAX_ROBOCLOAW;
    setpoint = map(knob_input, 0, 1023, map_min, map_max);
  }

  Serial.print("Setpoint Value: ");
  Serial.println(setpoint);
  MotorController.writeMicroseconds(setpoint);
  mode=WIRELESS;
  delay(20);
}