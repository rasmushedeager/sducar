/*

  SDU CAR - SELF TESTER CODE... WILL RUN EACH FUNCTION TO TEST IF IT WORKS.

  CHECK SERIAL MONITOR FOR FEEDBACK

*/

#include <SDU_CAR.h>

CAR car;
DATA cardata;
LOG carlog;

void setup() {
  Serial.begin(115200);
  Serial.println("Test starting...");
  cardata.begin();
  carlog.begin();

  Serial.println("Car initalized...");
}

void loop() {
  // Starting tester code:

  Serial.println("Runninger motors in 5 seconds...");
  Serial.println("2 seconds forwards, 2 seconds backwards at 100% speed.");
  delay(5000);
  car.setCarSpeed(100,100);
  delay(2000);
  car.setCarSpeed(-100,-100);
  delay(2000);
  car.setCarSpeed(0,0);
  delay(2000);

  
  Serial.println("Printing amount of ticks from each tacho sensor:");
  delay(2000);
  Serial.println("\nLeft tacho sensor: " + String(cardata.getTachoLeft()) + " - Right sensor: " + String(cardata.getTachoRight()));
  delay(2000);


  Serial.println("Battery sensor test...");
  delay(2000);
  Serial.println("Current battery voltage: " + String(cardata.getBatteryVoltage()) + " V");
  delay(2000);

  
  Serial.println("Accelerometer test...");
  delay(2000);
  for(int i = 0; i<10;i++) {
    cardata.readAccel();
    Serial.println("x: " + String(cardata.getAccel(x)) + " m/s^2\ty: " + String(cardata.getAccel(y)) + " m/s^2\tz: " + String(cardata.getAccel(z)) + " m/s^2");
    delay(500);
  }
  delay(2000);

  
  Serial.println("Line Sensor test...");
  delay(2000);
  for(int i = 0; i<10;i++) {
    Serial.println("Sensor 1: " + String(cardata.getLineSensor(1)) + "\tSensor 2: " + String(cardata.getLineSensor(2)) + "\tSensor 3: " + 
      String(cardata.getLineSensor(3)) + "\tSensor 4: " + String(cardata.getLineSensor(4)) + "\tSensor 5: " + String(cardata.getLineSensor(5)));
      delay(500);
  }
  delay(2000);

  Serial.println("Latch test...");
  Serial.println("Will flash each led 2 times starting from LSB to MSB");
  delay(2000);
  for(int i = 0; i<8;i++) {
    car.setLatch((1<<i));
    delay(1000);
  }
  for(int i = 0; i<8;i++) {
    car.setLatch((1<<i));
    delay(1000);
  }
  car.setLatch(0x00);
  delay(2000);

  Serial.println("Logging test...");
  delay(2000);
  carlog.log("LOG TEST...");
  delay(2000);

  Serial.println("TEST COMPLETED!");
  
}
