#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include "Wire.h"
#include "LittleFS.h"

// ======================= WiFi ========================== //
WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on

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

// ======================= ADS1115 ======================= //
#define ADS1115_ADDRESS  0x48
#define ADS_CONVERSION   0x00
#define ADS_CONFIG       0x01
#define ADS_LO_THRESH    0x02
#define ADS_HI_THRESH    0x03

// ====================== PINOUT ======================== //
#define RED_PIN 14
#define GREEN_PIN 2
#define BLUE_PIN 0
#define MPU_INT 12
#define ADC_INT 13

// ======================== LED ========================= //
uint8_t seconds_elapsed;
bool wake_status;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}
// ======================== ADC ========================= //
inline bool in_range(int16_t value, int16_t min_value, int16_t max_value) {
  return ((value > min_value) && (value < max_value));
}

uint8_t adc_active_channel;

const uint8_t cfg1_ch0 = 0b01000000;
const uint8_t cfg1_ch1 = 0b01010000;
const uint8_t cfg1_ch2 = 0b01100000;
const uint8_t cfg1_ch3 = 0b01110000;
const uint8_t cfg2     = 0b10001001;

const uint8_t ch_cfg[] = {
  cfg1_ch2,
  cfg1_ch3,
  cfg1_ch0,
  cfg1_ch1
};

volatile bool adcstate;
void IRAM_ATTR adcint() {
  adcstate = true;
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
#define FC_STREAM   4
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

bool stream_status;
// ==================== DATA REGISTERS ===================== //
#define REG_SIZE          64
#define REG_EEPROM_BEGIN  0x16
#define REG_EEPROM_END    64

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
int16_t* btnFlags = (int16_t*)&data_reg[REG_FLAGS];
int16_t* rawADC   = (int16_t*)&data_reg[REG_JOY];
int16_t* gyroBias = (int16_t*)&data_reg[REG_GBIAS];
int16_t* magBias  = (int16_t*)&data_reg[REG_MBIAS];
int16_t* btnSetup = (int16_t*)&data_reg[REG_BTN1];
uint8_t* led      = (uint8_t*)&data_reg[REG_LED];
float* accel      = (float*)&data_reg[REG_ACC];
float* gyro       = (float*)&data_reg[REG_GYRO];
float* mag        = (float*)&data_reg[REG_MAG];
char* ssid        = (char*)&data_reg[REG_SSID];
char* pass        = (char*)&data_reg[REG_PASS];

const char* fs_path = "/regdata.txt";

// =================== OTHER GLOBALS ==================== //
unsigned long ms_last, ms_accum, ms_parseUDP_accum;
uint16_t imu_counter, mag_counter, loop_counter, adc_counter;

const float gyro_conversion = (PI / 180.0f) * 1000.0f / 32768.0f;
const float acc_conversion  = 2.0f * 9.81f / 32768.0f;
const float mag_conversion  = 4912.0f / 32760.0f;

// Inner axis orientation
// Set index and sign of each axis
uint8_t x_i = 0,
        y_i = 2,
        z_i = 1;
int8_t  x_s = 1,
        y_s = -1,
        z_s = 1;

// ======================= SETUP ======================== //
void setup() {
  Serial.begin(115200);
  delay(1000);
  // Clear registers
  Serial.println("Initializing");
  memset((char*)(data_reg), '\0',  4 * REG_SIZE);
  /*
    // Init registers from eeprom
    EEPROM.begin(get_eeprom_addr(REG_SIZE));
    for (int addr = REG_EEPROM_BEGIN; addr < REG_EEPROM_END; ++addr) {
      EEPROM.get(get_eeprom_addr(addr), data_reg[addr]);
    }
  */
  // init with little fs
  Serial.println("Formatting LittleFS filesystem");
  LittleFS.format();
  if (LittleFS.begin()) {
    Serial.println("LittleFS mount success");
    readFile(fs_path);
  } else {
    Serial.println("LittleFS mount failed");
  }


  // Setup LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Set LED color to blue when device is attempting to connect
  setColor(0, 0, 255);

  // Setup comm
  Wire.begin();
  Wire.setClock(400000);


  // Init MPU
  Serial.println("Initializing MPU9250");
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

  // Init ADC
  pinMode(ADC_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ADC_INT), adcint, FALLING);
  adc_active_channel = 0;
  writeByte2(ADS1115_ADDRESS, ADS_CONFIG, ch_cfg[adc_active_channel], cfg2);
  writeByte2(ADS1115_ADDRESS, ADS_LO_THRESH, 0x00, 0x00);
  writeByte2(ADS1115_ADDRESS, ADS_HI_THRESH, 0b10000000, 0x00);

  // Init WiFi
  Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
  Serial.println("Starting wifi.");
  WiFi.begin();
  wifi_set_sleep_type(NONE_SLEEP_T);
  Serial.printf("WiFi connecting");
  int wifi_connect_counter = 0;
  int n_max_retry = 40;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    ++wifi_connect_counter;
    if (wifi_connect_counter > n_max_retry)
    { //------------------------------------------------------------- WiFi Failure Mode
      Serial.println("\nWiFi Failure mode:");
      Serial.print("old ssid:"); Serial.println(ssid);
      Serial.print("old pass:"); Serial.println(pass);
      bool blink_status = false;
      int login_step = 0;
      unsigned long tstart = millis();
      while (1) {
        unsigned long elapsed = millis() - tstart;
        if (elapsed > 1000) {
          blink_status = !blink_status;
          setColor((blink_status ? 255 : 0), 0, 0);
          if (login_step == 0) Serial.println("Enter SSID.");
          if (login_step == 1) Serial.println("Enter password.");
          if (login_step == 2) {
            Serial.println("WiFi credentials accepted.");
            for (int addr = REG_SSID; addr != REG_SIZE; ++addr) EEPROM.put(get_eeprom_addr(addr), data_reg[addr]);
            //EEPROM.commit();
            //delay(1000);
            //ESP.restart();
            Serial.println("Connecting again with");
            Serial.println(pass);
            Serial.println(ssid);
            WiFi.persistent(true);
            WiFi.begin(ssid, pass);
            wifi_connect_counter = 0;
            break;

          }
          tstart = millis();
        }
        if (Serial.available() > 0) {
          char incoming[32] = {'\0'};
          int bytes_read = Serial.readBytes(incoming, sizeof(incoming));
          incoming[bytes_read - 1] = '\0';
          Serial.println(incoming);
          if (login_step == 0) memcpy(ssid, incoming, 32);
          if (login_step == 1) memcpy(pass, incoming, 32);
          ++login_step;
        }
        delay(50);
      } //----------------------------------------------------------- WiFi Failure Mode
    }
  }
  Serial.println("\nWiFi connected succesfully!");
  delay(100);
  //Serial.end();

  // Begin udp
  setColor(0, 255, 0);
  Udp.begin(localUdpPort);

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
    accel[0]  = x_s * rawAcc[x_i] * acc_conversion;
    accel[1]  = y_s * rawAcc[y_i] * acc_conversion;
    accel[2]  = z_s * rawAcc[z_i] * acc_conversion;
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
    gyro[0]    = x_s * rawGyro[x_i] * gyro_conversion;
    gyro[1]    = y_s * rawGyro[y_i] * gyro_conversion;
    gyro[2]    = z_s * rawGyro[z_i] * gyro_conversion;

    ++imu_counter;

    if (stream_status) {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write('1');
      Udp.write((uint8_t*)accel, 12);
      Udp.write((uint8_t*)gyro, 12);
      Udp.endPacket();
      //delay(10);
    }
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
      /*
        mag[0]   =  y_s * rawMag[y_i] * mag_conversion;
        mag[1]   =  x_s * rawMag[x_i] * mag_conversion;
        mag[2]   = -z_s * rawMag[z_i] * mag_conversion;
      */
      mag[0]   = rawMag[1] * mag_conversion;
      mag[1]   = rawMag[2] * mag_conversion;
      mag[2]   = rawMag[0] * mag_conversion;
      ++mag_counter;

      if (stream_status) {
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write('2');
        Udp.write((uint8_t*)mag, 12);
        Udp.endPacket();
        //delay(10);
      }
    }
  }

  // check ADC data ready
  // adcstate = digitalRead(ADC_INT);
  if (adcstate) {
    uint8_t adcBuffer[2] = {0};
    readBytes(ADS1115_ADDRESS, ADS_CONVERSION, 2, &adcBuffer[0]);
    rawADC[adc_active_channel] = ((int16_t)adcBuffer[0] << 8) | adcBuffer[1];

    if (stream_status) {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write('3');
      Udp.write((uint8_t*)rawADC, 8);
      Udp.endPacket();
      //delay(10);
    }

    ++adc_active_channel;
    if (adc_active_channel > 3) adc_active_channel = 0;
    writeByte2(ADS1115_ADDRESS, ADS_CONFIG, ch_cfg[adc_active_channel], cfg2);
    ++adc_counter;
    adcstate = false;
  }

  // Set button flags
  if (in_range(rawADC[3], btnSetup[0], btnSetup[1])) btnFlags[0] |= 0b00000001;
  if (in_range(rawADC[3], btnSetup[2], btnSetup[3])) btnFlags[0] |= 0b00000010;
  if (in_range(rawADC[3], btnSetup[4], btnSetup[5])) btnFlags[0] |= 0b00000100;
  if (in_range(rawADC[3], btnSetup[6], btnSetup[7])) btnFlags[0] |= 0b00001000;

  // Get timing
  unsigned long ms_now = millis();
  unsigned long elapsed = ms_now - ms_last;
  ms_last = ms_now;
  // accumulate elapsed time
  ms_accum += elapsed;
  ms_parseUDP_accum += elapsed;

  // Count sample rate
  if (ms_accum > 1000) {
    ++seconds_elapsed;
    rawAcc[3] = imu_counter;
    imu_counter = 0;
    rawMag[3] = mag_counter;
    mag_counter = 0;
    rawGyro[3] = loop_counter;
    loop_counter = 0;
    btnFlags[1] = adc_counter;
    adc_counter = 0;
    ms_accum = 0;
  }

  // Sleep LED
  if (!stream_status) {
    if (wake_status & (seconds_elapsed > led[3]) ) {
      setColor(0, 0, 0);
      wake_status = false;
    }
  }

  if (ms_parseUDP_accum > 100) {
    // Listen for incoming UDP packets
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      char buf[64] = {0};
      Udp.read(buf, sizeof(buf));

      // Run protocol
      bool error = (buf[MSG_ADDR] + buf[MSG_LEN]) > REG_SIZE;
      if (!error) {
        switch (buf[MSG_FC]) {
          case FC_ZERO:
            break;
          case FC_READ:
            writeUDP((uint8_t*)(data_reg + buf[MSG_ADDR]), 4 * buf[MSG_LEN]);
            break;
          case FC_WRITE:
            memcpy((char*)(data_reg + buf[MSG_ADDR]), (buf + MSG_DATA), 4 * buf[MSG_LEN]);
            break;
          case FC_COMMIT:
            for (int addr = REG_EEPROM_BEGIN; addr != REG_EEPROM_END; ++addr) {
              EEPROM.put(get_eeprom_addr(addr), data_reg[addr]);
            }
            delay(100);
            /*
              Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

              if (EEPROM.commit()) {
              Udp.write("Commit success");

              } else {
              Udp.write("Commit failed");
              }


              Udp.endPacket();
            */
            writeFile(fs_path, "Hello fs");
            break;
          case FC_STREAM:
            stream_status = !stream_status;
            //stream_address = Udp.remoteIP();
            break;
        }
      }
      // wake led
      setColor(led[0], led[1], led[2]);
      seconds_elapsed = 0;
      wake_status = true;
      // reset button flags
      btnFlags[0] = 0;
    }
    ms_parseUDP_accum = 0;
  }
  if (stream_status) delay(10);
}

// ======================= READ / WRITE ========================= //
void writeUDP(uint8_t* buf, size_t len) {
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(buf, len);
  Udp.endPacket();
}
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void writeByte2(uint8_t address, uint8_t subAddress, uint8_t data1, uint8_t data2)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data1);
  Wire.write(data2);
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

// fs functions
void readFile(const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = LittleFS.open(path, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  delay(2000);  // Make sure the CREATE and LASTWRITE times are different
  file.close();
}
