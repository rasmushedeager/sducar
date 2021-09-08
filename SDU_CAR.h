/* SDU CAR library
created by Rasmus Hedeager Mikkelsen 
ver 1.1
*/
#ifndef SDU_CAR_h
#define SDU_CAR_h

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define avg_data_points 10

// Line Follower sensor pins
#define lfs_5_pin A0
#define lfs_4_pin A1
#define lfs_3_pin 8
#define lfs_2_pin A2
#define lfs_1_pin A3

// Extention board pins
#define ext_board_analog_pin A6
#define ext_board_digital_pwm_pin 9

// Sensor pins
#define battery_sens_pin A7
#define tacho_left_pin 2
#define tacho_right_pin 3

// H-Bridge motor controller pins
#define ml_dir_pin 4
#define ml_speed_pin 5
#define mr_dir_pin 7
#define mr_speed_pin 6

// Car paramerters for data calculation
#define slots_in_opto_wheel 20
#define wheel_circum 0.20735 //in meters
#define batt_voltage_multiplier 0.00722656


//////////////////////////////////////////////////////// ACCELOROMETER ///////////////////////////////////////////////////////////////

#define MMA8451__ADDRESS 0x1D

#define MMA8451_REG_OUT_X_MSB 0x01 //!< Read-only device output register
#define MMA8451_REG_SYSMOD 0x0B    //!< SYSMOD system mode register
#define MMA8451_REG_WHOAMI 0x0D    //!< WHO_AM_I device ID register
#define MMA8451_REG_XYZ_DATA_CFG 0x0E //!< XYZ_DATA_CFG register, sets dynamic range and high-pass filter for
       //!< output data
#define MMA8451_REG_PL_STATUS 0x10 //!< PL_STATUS portrait/landscape status register
#define MMA8451_REG_PL_CFG 0x11 //!< Portrait/landscape configuration register
#define MMA8451_REG_CTRL_REG1 0x2A //!< CTRL_REG1 system control 1 register
#define MMA8451_REG_CTRL_REG2 0x2B //!< CTRL_REG2 system control 2 register
#define MMA8451_REG_CTRL_REG4 0x2D //!< CTRL_REG4 system control 4 register
#define MMA8451_REG_CTRL_REG5 0x2E //!< CTRL_REG5 system control 5 register

typedef enum {
  MMA8451_RANGE_8_G = 0b10, // +/- 8g
  MMA8451_RANGE_4_G = 0b01, // +/- 4g
  MMA8451_RANGE_2_G = 0b00  // +/- 2g (default value)
} mma8451_range_t;

typedef enum {
  x = 1, // +/- 8g
  y = 2, // +/- 4g
  z = 3  // +/- 2g (default value)
} accel_data_dir_t;

/*! Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum {
  //MMA8451_DATARATE_800_HZ = 0b000,  //  800Hz
  //MMA8451_DATARATE_400_HZ = 0b001,  //  400Hz
  MMA8451_DATARATE_200_HZ = 0b010,  //  200Hz
  //MMA8451_DATARATE_100_HZ = 0b011,  //  100Hz
  //MMA8451_DATARATE_50_HZ = 0b100,   //   50Hz
  //MMA8451_DATARATE_12_5_HZ = 0b101, // 12.5Hz
  //MMA8451_DATARATE_6_25HZ = 0b110,  // 6.25Hz
  //MMA8451_DATARATE_1_56_HZ = 0b111, // 1.56Hz

  MMA8451_DATARATE_MASK = 0b111
} mma8451_dataRate_t;

#define SENSORS_GRAVITY_STANDARD (9.816F)
////////// END ////////////


////////////////////////////////////////////////////// LATCH CODE ///////////////////////////////////////////////////////
#define LATCH_ADR 0x70 //(current = write, +1 for read)

void Tacho_left(void);
void Tacho_right(void);

class CAR
  { 
    private: // Library specific internal variables
    
            
    public: // Functions called inside the library
      void begin(void);
      void setLatch(uint8_t lightByte);
      void setCarSpeed(int left_speed, int right_speed);
  };

class DATA
  { 
    private: // Library specific internal variables
            int rand_var;
            int32_t _sensorID;
            int8_t _i2caddr;
            int16_t x, y, z;
            float x_g, y_g, z_g;
            //Averager:
            int avg_count = 0;
            int x_avg[avg_data_points], y_avg[avg_data_points], z_avg[avg_data_points];
            int a11, a12, a13, a21, a22, a23, a31, a32, a33; // Matrix values
            float memsOffset[3];
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;
            int lineSensor[5];

            // Private functions:
            void setDataRate(mma8451_dataRate_t dataRate);
            mma8451_range_t getRange(void);
            bool beginAccel(uint8_t addr = MMA8451__ADDRESS);
            void writeRegister8(uint8_t reg, uint8_t value);
            uint8_t readRegister8(uint8_t reg);
            void enableTacho(void);
            float getRawAccel(accel_data_dir_t dir);
            
    public: // Functions called inside the library
      void begin(void);
      float t(void);
      void calibrateMEMS(void);
      void readAccel(void);
      float getAccel(accel_data_dir_t dir);
      //float getFilteredAccel(accel_data_dir_t dir);
      void resetTacho(void);
      unsigned int getTachoLeft(void);
      unsigned int getTachoRight(void);
      float getDistLeft(void);
      float getDistRight(void);
      float getBatteryVoltage(void);
      void readLineSensor(void);
      int getLineSensor(char sensor_number);
  };

class LOG
  { 
    private: // Library specific internal variables
            File logfile;
            int counter = 0;
            void flushCard(void);
            uint8_t LOG::getFileCount(void);
    public: // Functions called inside the library
      void begin(void);
      void log(const String& logdata);
  };

#endif
