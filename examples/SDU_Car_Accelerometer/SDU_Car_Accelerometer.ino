/*

  SDU CAR - ACCELEROMETER WORKSHOP

  Written by Rasmus Hedeager Mikkelsen
  AT SDU 19-08-2021

*/

#include "SDU_CAR.h"


#define DRIVINGDISTANCE 0.5 // In meters

#define DRIVINGSPEED 60  // In percent


// Sets up the SDU Car library.
CAR car;
DATA cardata;
LOG carlog;

void setup() {
  //Serial.begin(9600);
  // Enables the use of motors.
  car.begin();

  // Enables the use of sensors.
  cardata.begin();
  // Enables the use of SD Card.
  carlog.begin();

  carlog.log("t\ta\td");
}

int logDelay = 0;

void loop() {

  float distDrivenR = cardata.getDistRight();
  float distDrivenL = cardata.getDistLeft();

  float delta = cardata.getTachoRight() - cardata.getTachoLeft();

  float avgDist = ( distDrivenR + distDrivenL ) / 2;

  cardata.readAccel();
  //Serial.println(String(cardata.getAccel(x), 3) + "\t" + String(cardata.getAccel(y), 3) + "\t" + String(cardata.getAccel(z), 3));

  float xVal = cardata.getAccel(x);


  if( avgDist < DRIVINGDISTANCE ) {
    
    if(logDelay < 40) {
      logDelay++;
    } else {
      car.setCarSpeed(DRIVINGSPEED + (10 * delta) ,DRIVINGSPEED - (10 * delta));
      carlog.log( String(cardata.t(), 5) + "\t" + String(xVal, 3) + "\t" + String(avgDist, 3));
    }
    
  } else {
    
    car.setCarSpeed(0,0);
    
    if(logDelay < 80) {
      carlog.log( String(cardata.t(), 5) + "\t" + String(xVal, 3) + "\t" + String(avgDist, 3));
      logDelay++;
    }
    
  }
  
}
