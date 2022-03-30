#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;


struct data_send{
  long int leftencoder;
  long int rightencoder;
  long int battery_mV;
};                          // The Struct of data we would like to send to Pi, contains 12 Bytes of data;


struct data_read{
  long int leftspd;
  long int rightspd;
};                           // The struct of data we recieve, contains 8 bytes of data;

union tx{
  struct data_send data;
  byte byte_send[12];
};

union rx {
  struct data_read data;
  byte byte_read[8];
};


union rx info_read;
union tx info_send;


unsigned long int time_start;
unsigned long int time_end;

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // when master send information, call void function "recieveEvent"
  Wire.onRequest(requestEvent); // when master asks for information, call void function "request Event"
  Serial.begin(9600);           // start serial for output to PC monitor - used solely for troubleshooting
  info_send.data.leftencoder = encoders.getCountsAndResetLeft();      //reset count registers
  info_send.data.rightencoder = encoders.getCountsAndResetRight();
  delay(5000);
  Serial.println("READY");
}

void loop() {
      info_send.data.leftencoder += encoders.getCountsAndResetLeft();
      info_send.data.rightencoder += encoders.getCountsAndResetRight();
      info_send.data.battery_mV = readBatteryMillivolts();
	  buzzer.playNote(NOTE_A(4), 3, 15); //Make sound while driving
}

void receiveEvent(int howMany) {
  int i = 0;                    // Careful of indexing, through testing the Pi will send MSB --> LSB. Some I2C devices do it the opposite
  while (Wire.available()){     // Wire.available returns the int value of number of bytes, each iteration reads the next byte.
    info_read.byte_read[i] = Wire.read();
    i++;
  }
  motors.setSpeeds(info_read.data.leftspd,info_read.data.rightspd);
}

void requestEvent() {
  int j =0;                     // Cannot simply write the integer, it will send in the wrong order - you must index from MSB to LSB. Remeber that I2C is sending a BYTE at a time.
  for (j=0;j<12;j++){
      Wire.write(info_send.byte_send[j]);
  }
  info_send.data.leftencoder = 0;
  info_send.data.rightencoder = 0;
}
 

