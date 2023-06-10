#include "Wire.h"
#include <EEPROM.h>

// ====================== MPU9250 ======================== //
#define MPU9250_ADDRESS  0x68
#define WHO_AM_I_MPU9250 0x75

#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A

#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A

#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40

#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48

#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define I2C_MST_CTRL     0x24

#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09
#define AK8963_CNTL      0x0A

// ====================== PINOUT ======================== //
#define RED_PIN 9
#define GREEN_PIN 10
#define BLUE_PIN 11
#define MPU_INT 2

// ======================== LED ========================= //
uint8_t seconds_elapsed;
bool wake_status;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

// ====================== PROTOCOL ======================= //
// Message bytes
#define MSG_FC      0
#define MSG_ADDR    1
#define MSG_LEN     2
#define MSG_DATA    4

// Function codes
#define FC_ZERO     0
#define FC_READ     1
#define FC_WRITE    2
#define FC_ERROR    3
#define FC_COMMIT   5

// Read only register map
#define REG_ACC     0x00
#define REG_GYRO    0x03
#define REG_MAG     0x06
#define REG_FLAGS   0x09
#define REG_JOY     0x0A
#define REG_TRGBTN  0x0B
#define REG_RAWA    0x0C
#define REG_RAWG    0x0E
#define REG_RAWM    0x10

// Read + Write register map
#define REG_LED     0x16
#define REG_YAWOFS  0x17
#define REG_NAME    0x18
#define REG_MBIAS   0x20
#define REG_GBIAS   0x22
#define REG_FILTB   0x24
#define REG_FILTZ   0x25

#define REG_JMIN    0x28
#define REG_JMAX    0x29
#define REG_JMID    0x2A
#define REG_TRIG    0x2B
#define REG_BTN1    0x2C
#define REG_BTN2    0x2D
#define REG_BTN3    0x2E
#define REG_BTN4    0x2F
#define REG_SSID    0x30
#define REG_PASS    0x38

// ==================== DATA REGISTERS ===================== //
#define REG_SIZE          64
#define REG_EEPROM_BEGIN  0x16
#define REG_EEPROM_END    0x25

unsigned int get_eeprom_addr(unsigned int addr) {
  if (addr < REG_EEPROM_BEGIN) addr = REG_EEPROM_BEGIN;
  if (addr > REG_EEPROM_END) addr = REG_EEPROM_END;
  return 4 * (addr - REG_EEPROM_BEGIN);
}

// data registers
float data_reg[REG_SIZE];

// Pointers to register data
int16_t* rawAcc   = (int16_t*)&data_reg[REG_RAWA];
int16_t* rawGyro  = (int16_t*)&data_reg[REG_RAWG];
int16_t* rawMag   = (int16_t*)&data_reg[REG_RAWM];
int16_t* gyroBias = (int16_t*)&data_reg[REG_GBIAS];
int16_t* magBias  = (int16_t*)&data_reg[REG_MBIAS];
uint8_t* led      = (uint8_t*)&data_reg[REG_LED];
float* gyro       = (float*)&data_reg[REG_GYRO];
float* accel      = (float*)&data_reg[REG_ACC];
float* mag        = (float*)&data_reg[REG_MAG];

// =================== OTHER GLOBALS ==================== //
unsigned long ms_last, ms_accum;
uint16_t imu_counter, mag_counter, loop_counter;

const float gyro_conversion = (PI / 180.0f) * 1000.0f / 32768.0f;
const float acc_conversion  = 2.0f * 9.81f / 32768.0f;
const float mag_conversion  = 4912.0f / 32760.0f;

// ======================= SETUP ======================== //
void setup() {
  // Clear registers
  memset((char*)&data_reg[0], '\0',  4 * REG_SIZE);

  // Init registers from eeprom
  for (int addr = REG_EEPROM_BEGIN; addr < REG_EEPROM_END; ++addr) {
    EEPROM.get(get_eeprom_addr(addr), data_reg[addr]);
  }

  // Setup LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Setup comm
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Init MPU
  pinMode(MPU_INT, INPUT);
  digitalWrite(MPU_INT, LOW);
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x73)
  {
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    delay(100);
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0b00010000);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x00);
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
    delay(100);
  }

  // Init magnetometer
  byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
  if (d == 0x48) {
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    delay(100);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x16);
    delay(100);
  }

  // Init sample rate counting
  imu_counter = 0;
  mag_counter = 0;
  loop_counter = 0;
  ms_last = millis();
  ms_accum = 0;

  // Init LED state
  seconds_elapsed = 0;
  wake_status = true;
}

// ======================= LOOP ========================= //
void loop() {
  ++loop_counter;

  // check MPU data ready
  bool imu_data_ready = readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
  if (imu_data_ready) {
    // read accelerometer bytes
    uint8_t accBuffer[6];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &accBuffer[0]);
    rawAcc[0] = ((int16_t)accBuffer[0] << 8) | accBuffer[1];
    rawAcc[1] = ((int16_t)accBuffer[2] << 8) | accBuffer[3];
    rawAcc[2] = ((int16_t)accBuffer[4] << 8) | accBuffer[5];
    // Convert data
    accel[0]  = -rawAcc[0] * acc_conversion;
    accel[1]  = -rawAcc[2] * acc_conversion;
    accel[2]  = -rawAcc[1] * acc_conversion;
    
    // read gyroscope bytes
    uint8_t gyroBuffer[6];
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &gyroBuffer[0]);
    rawGyro[0] = ((int16_t)gyroBuffer[0] << 8) | gyroBuffer[1];
    rawGyro[1] = ((int16_t)gyroBuffer[2] << 8) | gyroBuffer[3];
    rawGyro[2] = ((int16_t)gyroBuffer[4] << 8) | gyroBuffer[5];
    // Bias offset
    rawGyro[0] -= gyroBias[0];
    rawGyro[1] -= gyroBias[1];
    rawGyro[2] -= gyroBias[2];
    // Convert data
    gyro[0]    = -rawGyro[0] * gyro_conversion;
    gyro[1]    = -rawGyro[2] * gyro_conversion;
    gyro[2]    = -rawGyro[1] * gyro_conversion;
    // increment counter
    ++imu_counter;
  }

  // Check magnetometer data ready
  bool mag_data_ready = readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01;
  if (mag_data_ready) {
    // ST2 register must be read ST2 at end of data acquisition
    uint8_t magBuffer[7];
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &magBuffer[0]);
    // Check magnetic sensor overflow bit is not set
    mag_data_ready = !(magBuffer[6] & 0x08);
    if (mag_data_ready) {
      // Transfer data from buffer
      rawMag[0] = ((int16_t)magBuffer[1] << 8) | magBuffer[0] ;
      rawMag[1] = ((int16_t)magBuffer[3] << 8) | magBuffer[2] ;
      rawMag[2] = ((int16_t)magBuffer[5] << 8) | magBuffer[4] ;
      // Bias offset
      rawMag[0] -= magBias[0];
      rawMag[1] -= magBias[1];
      rawMag[2] -= magBias[2];
      // Convert data
      mag[0]   =  -rawMag[1] * mag_conversion;
      mag[1]   =  rawMag[2] * mag_conversion;
      mag[2]   =  -rawMag[0] * mag_conversion;
      ++mag_counter;
    }
  }

  // Get timing
  unsigned long ms_now = millis();
  unsigned long elapsed = ms_now - ms_last;
  ms_last = ms_now;
  // accumulate elapsed time
  ms_accum += elapsed;

  // Count sample rate
  if (ms_accum > 1000) {
    ++seconds_elapsed;
    rawAcc[3] = imu_counter;
    imu_counter = 0;
    rawMag[3] = mag_counter;
    mag_counter = 0;
    rawGyro[3] = loop_counter;
    loop_counter = 0;
    ms_accum = 0;
  }

  // Sleep LED
  if (wake_status & (seconds_elapsed > led[3]) ) {
    setColor(0, 0, 0);
    wake_status = false;
  }

  // Read data from serial port
  if (Serial.available()) {
    char buf[64] = {0};
    uint8_t i = 0;
    while (Serial.available()) {
      char c = Serial.read();
      if (c != '\n' && i < sizeof(buf))
        buf[i] = c;
      i++;
    }

    // Run protocol
    bool error = (buf[MSG_ADDR] + buf[MSG_LEN]) > REG_SIZE;
    if (!error) {
      switch (buf[MSG_FC]) {
        case FC_ZERO:
          break;
        case FC_READ: // read
          Serial.write((char*)(data_reg + buf[MSG_ADDR]), 4 * buf[MSG_LEN]);
          break;
        case FC_WRITE: // write
          memcpy((char*)(data_reg + buf[MSG_ADDR]), (buf + MSG_DATA), 4 * buf[MSG_LEN]);
          break;
        case FC_COMMIT: // commit
          for (int addr = REG_EEPROM_BEGIN; addr != REG_EEPROM_END; ++addr) {
            EEPROM.put(get_eeprom_addr(addr), data_reg[addr]);
          }
          break;
      }
    }
    // wake led
    setColor(led[0], led[1], led[2]);
    seconds_elapsed = 0;
    wake_status = true;
  }
  // add some loop delay
  yield();
  delay(5);
}

// ======================= READ / WRITE ========================= //
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t) 1);
  data = Wire.read();
  return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(address, count);
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }
}
