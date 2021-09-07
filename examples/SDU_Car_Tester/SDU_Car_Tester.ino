/*

  SDU CAR - SELF TESTER CODE... WILL RUN EACH FUNCTION TO TEST IF IT WORKS.

  CHECK SERIAL MONITOR FOR FEEDBACK

*/

#include <SDU_CAR.h>

CAR car;
DATA cardata;
LOG carlog;

void setup() {
  Serial.begin(9600);
  Serial.println("Test starting...");
  car.begin();
  Serial.println("Latch and motors initialized...");
  cardata.begin();
  Serial.println("Data communication initalized...");
  carlog.begin();
}


void loop() {
  // Starting tester code:
  delay(3000);
  Serial.println("Testing motors...");
  car.setCarSpeed(50,50);
  delay(1500);
  car.setCarSpeed(100,100);
  delay(1500);
  car.setCarSpeed(-50,-50);
  delay(1500);
  car.setCarSpeed(-100,-100);
  delay(1500);
  car.setCarSpeed(0,0);

  Serial.println("Battery voltage: " + String(cardata.getBatteryVoltage(), 2) + "v");
  delay(5000);
  Serial.println("Tacho Left: " + String(cardata.getTachoLeft()) + " - Tacho Right: " + String(cardata.getTachoRight()));
  delay(5000);
  cardata.readAccel();
  Serial.println("Accelerometer data: x: " + String(cardata.getAccel(x), 4) + " - y: " + String(cardata.getAccel(y), 4)+ " - z: " + String(cardata.getAccel(z), 4));
  delay(5000);
  cardata.readLineSensor();
  Serial.println("Line sensor 1: " + String(cardata.getLineSensor(1)));
  Serial.println("Line sensor 2: " + String(cardata.getLineSensor(2)));
  Serial.println("Line sensor 3: " + String(cardata.getLineSensor(3)));
  Serial.println("Line sensor 4: " + String(cardata.getLineSensor(4)));
  Serial.println("Line sensor 5: " + String(cardata.getLineSensor(5)));
  Serial.println("Test Complete!");
  while(1);
}
