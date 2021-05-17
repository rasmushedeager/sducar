/* SDU CAR library
created by Rasmus Hedeager Mikkelsen 
ver 1.0
*/
#include "Arduino.h"
#include "SDU_CAR.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


volatile unsigned char left_count = 255;
volatile unsigned char left_ovf = 0;
volatile unsigned char right_count = 255;
volatile unsigned char right_ovf = 0;


void CAR::begin(){

  // initializes the pins for the motors:
  pinMode(ml_dir_pin,OUTPUT);
  pinMode(ml_speed_pin,OUTPUT);
  digitalWrite(ml_dir_pin,LOW);
  digitalWrite(ml_speed_pin,LOW);
    
  pinMode(mr_dir_pin,OUTPUT);
  pinMode(mr_speed_pin,OUTPUT);
  digitalWrite(mr_dir_pin,LOW);
  digitalWrite(mr_speed_pin,LOW);
  
  Wire.begin();
  Wire.beginTransmission(LATCH_ADR);
  Wire.write(0x00);
  Wire.endTransmission();
}


////////////////////// This section will control the tacho sensors and return functions //////////////////////////////
void INTtachoLeft() { //Called by external interrup when the tacho has a rising edge
  left_count++; // Add one to the tacho counter
  if(left_count == 255) { // Check for overflow - If there is an overflow add 1 to overflow var
    left_ovf++;
  }
}

void INTtachoRight() {
  right_count++;
  if(right_count == 255) {
    right_ovf++;
  }
}


void DATA::begin(void) {

  //Initializes the pins for the line follower sensor:
  pinMode(lfs_1_pin,INPUT);
  pinMode(lfs_2_pin,INPUT);
  pinMode(lfs_3_pin,INPUT);
  pinMode(lfs_4_pin,INPUT);
  pinMode(lfs_5_pin,INPUT);

  // Initializes the sensor pins including external interrupts for the tachometer
  pinMode(battery_sens_pin,INPUT);
  
  enableTacho();
  beginAccel();

  for(int i = 0; i<avg_data_points; i++) {
    readAccel();
    getFilteredAccel(x);
  }
}

void DATA::enableTacho() {
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), INTtachoLeft, FALLING);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), INTtachoRight, FALLING);
}

unsigned int DATA::getTachoLeft() {
  noInterrupts();
  unsigned char ovf = left_ovf;
  unsigned char tacho = left_count;
  interrupts();
  tacho++; //Add plus one to account for starting at 0 when there is an overflow.
  unsigned int value = ovf*256 + tacho;
  return value;
}

unsigned int DATA::getTachoRight() {
  noInterrupts();
  unsigned char ovf = right_ovf;
  unsigned char tacho = right_count;
  interrupts();
  tacho++; //Add plus one to account for starting at 0 when there is an overflow.
  unsigned int value = ovf*256 + tacho;
  return value;
}

void DATA::resetTacho(void) {    ///// NOTE ABOUT THE TACHO SENSOR: We have 256*256=65.536 data points, with 30 slots in the optocoulper wheel and and wheel diameter of 6cm - We can drive more than 300 meters before overflowing.
  noInterrupts();
  right_ovf = 0;
  right_count = 255;
  left_ovf = 0;
  left_count = 255;
  interrupts();
}

float DATA::getDistLeft() {
  unsigned int ticks = getTachoLeft();
  float value = ( float(ticks)/slots_in_opto_wheel ) * wheel_circum;
  return value;
}

float DATA::getDistRight() {
  unsigned int ticks = getTachoRight();
  float value = (float(ticks)/slots_in_opto_wheel) * wheel_circum;
  return value;
}
////////////////////// this function will return the status for the line line follower sensor called //////////////////////////////

bool DATA::getLineSensor(char sensor_number) {
  bool x = 0;
  switch(sensor_number) { // Takes the input number and runs the read based on what sensor is picked.
    case 1:
      x = digitalRead(lfs_1_pin);
    case 2:
      x = digitalRead(lfs_2_pin);
    case 3:
      x = digitalRead(lfs_3_pin);
    case 4:
      x = digitalRead(lfs_4_pin);
    default: 
      x = digitalRead(lfs_5_pin);
  }
  return x;
}


////////////////////// this function will read the analog pin and return the calculated voltage //////////////////////////////

float DATA::getBatteryVoltage(void) {
  int a_read = analogRead(battery_sens_pin); // Read the battry pin
  float calc = a_read * batt_voltage_multiplier; // Find the voltage using a coefficient
  return calc;
}

////////////////////// this function will control the motor speeds //////////////////////////////

 void CAR::setCarSpeed(int left_speed, int right_speed)   //This function will take the input speed in percent and automatically convert this value to use for the H-bridge.
  {
    int ls, rs, constr_l, constr_r;   // Creating variables
    if(left_speed < 0) {    // Checking to see if the speed is positive or negative
      digitalWrite(ml_dir_pin,LOW);   // Setting the correct direction
      constr_l = constrain(left_speed, -100, -1);   // Constraning the value within the desired range, ie.: -106% would return as -100%
      ls = map(constr_l, 0, -100, 0, 255);    // Mapping the speed to analog output of the arduino
      analogWrite(ml_speed_pin, ls);
    } else {                                                  
      digitalWrite(ml_dir_pin,HIGH);
      constr_l = constrain(left_speed, 0, 100);
      ls = map(constr_l, 0, 100, 0, 255);
      analogWrite(ml_speed_pin, (255 - ls));
    }

    if(right_speed < 0) {
      digitalWrite(mr_dir_pin,LOW);
      constr_r = constrain(right_speed, -100, -1);
      rs = map(constr_r, 0, -100, 0, 255);
      analogWrite(mr_speed_pin, rs);
    } else {
      digitalWrite(mr_dir_pin,HIGH);
      constr_r = constrain(right_speed, 0, 100);
      rs = map(constr_r, 0, 100, 0, 255);
      analogWrite(mr_speed_pin, (255 - rs));
    }
 }

//////////////////////////////////// ACCELROMETER - DO NOT MODIFY - BORROWED FROM https://www.arduino.cc/reference/en/libraries/adafruit-mma8451-library/ /////////////////////////////////////////////////

bool DATA::beginAccel(uint8_t i2caddr) {
  Wire.begin();
  _i2caddr = i2caddr;

  /* Check connection */
  uint8_t deviceid = readRegister8(MMA8451_REG_WHOAMI);
  if (deviceid != 0x1A) {
    /* No MMA8451 detected ... return false */
    // Serial.println(deviceid, HEX);
    return false;
  }

  writeRegister8(MMA8451_REG_CTRL_REG2, 0x40); // reset

  while (readRegister8(MMA8451_REG_CTRL_REG2) & 0x40)
    ;

  // enable 4G range
  writeRegister8(MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G);
  // High res
  writeRegister8(MMA8451_REG_CTRL_REG2, 0x02);
  // DRDY on INT1
  writeRegister8(MMA8451_REG_CTRL_REG4, 0x01);
  writeRegister8(MMA8451_REG_CTRL_REG5, 0x01);

  // Turn on orientation config
  writeRegister8(MMA8451_REG_PL_CFG, 0x40);

  // Activate at max rate, low noise mode
  writeRegister8(MMA8451_REG_CTRL_REG1, 0x01 | 0x04);

  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$");
    Serial.print(i, HEX); Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
  }
  */


  setDataRate(MMA8451_DATARATE_200_HZ);
  

  return true;
}

static inline uint8_t i2cread(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

static inline void i2cwrite(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

void DATA::writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_i2caddr);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value));
  Wire.endTransmission();
}

uint8_t DATA::readRegister8(uint8_t reg) {

// undocumented version of requestFrom handles repeated starts on Arduino Due
#ifdef __SAM3X8E__
  Wire.requestFrom(_i2caddr, 1, reg, 1, true);
#else
  // I don't know - maybe the other verion of requestFrom works on all
  // platforms.
  //  honestly, I don't want to go through and test them all.  Doing it this way
  //  is already known to work on everything else
  Wire.beginTransmission(_i2caddr);
  i2cwrite(reg);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!
  Wire.requestFrom(_i2caddr, 1);
#endif

  if (!Wire.available())
    return -1;
  return (i2cread());
}


mma8451_range_t DATA::getRange(void) {
  /* Read the data format register to preserve bits */
  return (mma8451_range_t)(readRegister8(MMA8451_REG_XYZ_DATA_CFG) & 0x03);
}

void DATA::setDataRate(mma8451_dataRate_t dataRate) {
  uint8_t ctl1 = readRegister8(MMA8451_REG_CTRL_REG1);
  writeRegister8(MMA8451_REG_CTRL_REG1, 0x00); // deactivate
  ctl1 &= ~(MMA8451_DATARATE_MASK << 3);       // mask off bits
  ctl1 |= (dataRate << 3);
  writeRegister8(MMA8451_REG_CTRL_REG1, ctl1 | 0x01); // activate
}

void DATA::readAccel(void) {
  // read x y z at once
  Wire.beginTransmission(_i2caddr);
  i2cwrite(MMA8451_REG_OUT_X_MSB);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!

  Wire.requestFrom(_i2caddr, 6);
  x = Wire.read();
  x <<= 8;
  x |= Wire.read();
  x >>= 2;
  y = Wire.read();
  y <<= 8;
  y |= Wire.read();
  y >>= 2;
  z = Wire.read();
  z <<= 8;
  z |= Wire.read();
  z >>= 2;

  uint8_t range = getRange();
  uint16_t divider = 1;
  if (range == MMA8451_RANGE_8_G)
    divider = 1024;
  if (range == MMA8451_RANGE_4_G)
    divider = 2048;
  if (range == MMA8451_RANGE_2_G)
    divider = 4096;

  x_g = (float)x / divider;
  y_g = (float)y / divider;
  z_g = (float)z / divider;

  //Serial.println(String( ((sqrt(  pow(x_g, 2) + pow(y_g, 2) + pow(z_g, 2)  ) * SENSORS_GRAVITY_STANDARD - SENSORS_GRAVITY_STANDARD + 0.16)) * multi, 4));
}

float DATA::getAccel(accel_data_dir_t dir) {
  switch(dir) {
    case 1: // ASCII for x
      return x_g * SENSORS_GRAVITY_STANDARD + first_x_offset;
    case 2: // ASCII for y
      return y_g * SENSORS_GRAVITY_STANDARD;
    case 3: // ASCII for z
      return z_g * SENSORS_GRAVITY_STANDARD;
    default:
      return 0.0;
  }
}

float DATA::getFilteredAccel(accel_data_dir_t dir) {

  x_avg[avg_count] = x_g * SENSORS_GRAVITY_STANDARD * 1000;
  y_avg[avg_count] = x_g * SENSORS_GRAVITY_STANDARD * 1000;
  z_avg[avg_count] = x_g * SENSORS_GRAVITY_STANDARD * 1000;
  
  if(avg_count == (avg_data_points - 1) ) {
    avg_count = 0;
    if(first_x_offset == 0) {
      float sum_offset = 0;
      for(int i = 0; i < avg_data_points; i++) {
        sum_offset += x_avg[i];
      }
      first_x_offset = 0 - sum_offset / (avg_data_points * 1000);
    }
  } else {
    avg_count++;
  }

  float sum = 0;
  
  switch(dir) {
    case 1: // ASCII for x
      for(int i = 0; i < avg_data_points; i++) {
        sum += x_avg[i];
      }
      //Serial.print("Offset: " + String(first_x_offset) + "\t X Averaged: ");
      //Serial.println(sum / (avg_data_points * 1000) + first_x_offset);
      return sum / (avg_data_points * 1000) + first_x_offset;
    case 2: // ASCII for y
      for(int i = 0; i < avg_data_points; i++) {
        sum += y_avg[i];
      }
      return sum / (avg_data_points * 1000);
    case 3: // ASCII for z
      for(int i = 0; i < avg_data_points; i++) {
        sum += z_avg[i];
      }
      return sum / (avg_data_points * 1000);
    default:
      return 0;
  }
}


//////////////////////////////////////////////////////////// SD CARD ///////////////////////////////////////////////////////////////////////

void LOG::begin(void) {
  if (!SD.begin(10)) {
    //Serial.println("initialization failed!");
    //while (1);
  }
  //Serial.println("initialization done.");

  logfile = SD.open("log_" + String(getFileCount()) + ".txt", FILE_WRITE);
}

void LOG::log(const String& logdata) {
  // if the file opened okay, write to it:
  if (logfile) {
    //Serial.print("Writing to log.txt...");
    logfile.println(logdata);
    // close the file:
    //logfile.close();
    flushCard();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial.println("error opening log.txt");
  }
}

uint8_t LOG::getFileCount(void) {
  File root = SD.open("/");
  uint8_t cnt = 0;
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    cnt++;
    entry.close();
  }
  return cnt;
}


void LOG::flushCard(void) {
  if (counter < 200) {
    counter++;
  } else {
    logfile.flush();
    counter = 0;
  }
}

//////////////////////////////////////////////////////// LATCH CODE //////////////////////////////////////////////////////////////////////

void CAR::setLatch(uint8_t lightByte) {
  Wire.beginTransmission(LATCH_ADR);
  Wire.write(lightByte);
  Wire.endTransmission();
}



 
