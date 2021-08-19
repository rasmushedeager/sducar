/*

  SDU CAR - BASIC LINE FOLLOWER CODE

  Written by Rasmus Hedeager Mikkelsen
  AT SDU 19-08-2021

*/

#include <SDU_CAR.h>



#define LIMIT 400 // Adjust this value to be in the middle of the output from the sensors, between surface and test strip.
                  // For example, at the surface the sensor returns 321, and on the test strip 543, 400 is choosen right in between.

// Sets up the SDU Car library.
CAR car;
DATA cardata;
LOG carlog;


void setup() {
  Serial.begin(9600); // Communication with the computer.

  // Enables the use of motors.
  car.begin();

  // Enables the use of sensors.
  cardata.begin();

  // Enables the use of SD Card.
  carlog.begin();
}


void loop() { // Only change the code below. The following code can be used and will work. However, it is NOT optimized.

  // Reads the line follower sensor.
  cardata.readLineSensor();

  //Checks if we are going straight and continues driving forwards.
  if(cardata.getLineSensor(3) > LIMIT) {
    car.setCarSpeed(30,30);
  }

  // Checks if we are going over the line and correcting the driving direction.
  if(cardata.getLineSensor(1) > LIMIT) {
    car.setCarSpeed(0,30);
  }

  if(cardata.getLineSensor(5) > LIMIT) {
    car.setCarSpeed(30,0);
  }


  // If you want to see the values the sensor returns, uncomment the line below by removing the // in the beginning of the line!
  
  //Serial.println(String(cardata.getLineSensor(1)) + "\t" + String(cardata.getLineSensor(2)) + "\t" + String(cardata.getLineSensor(3)) + "\t" + String(cardata.getLineSensor(4)) + "\t" + String(cardata.getLineSensor(5)));

  // The data can be seen by clicking the magnifying glass in the top right corner, while the car is plugged in.
  
}
