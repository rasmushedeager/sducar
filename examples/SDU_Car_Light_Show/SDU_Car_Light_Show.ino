/*

  SDU CAR - Light Show

  Written by Rasmus Hedeager Mikkelsen
  AT SDU 06-03-2022

*/

#include <SDU_CAR.h>

// Sets up the SDU Car library.
CAR car;
DATA cardata;
LOG carlog;

void setup() {
  Serial.begin(9600); // Communication with the computer.

  car.begin();
  cardata.begin();
  carlog.begin();
}


void loop() { // Include your code for controlling the line follower below:
  char lightChar = 0x01;
  for(int i=0;i<10;i++) {
    for(int y=0;y<8;y++) {
      car.setShiftReg(lightChar);
      delay(100);
      lightChar = (lightChar<<1);
    }
  }
  for(int i=0;i<10;i++) {
    car.setShiftReg(0xff);
    delay(100);
    car.setShiftReg(0x00);
    delay(100);
  }
  
}
