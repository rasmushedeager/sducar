/*

  SDU CAR - BASIC LINE FOLLOWER CODE

  Written by Rasmus Hedeager Mikkelsen
  AT SDU 25-08-2021

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

    
  
}
