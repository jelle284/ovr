/*
   Modified for raw data streaming
   added device discovery
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "Wire.h"

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
uint8_t led_timeout = 5;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}
// ======================== ADC ========================= //
int16_t adc_data[4];
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
// Message header, 4 bytes
#define MSG_FC      0 // function code
#define MSG_TYPE    1 // message type
#define MSG_LEN     2 // length of data
#define MSG_NUM     3 // packet number
#define MSG_DATA    4 // where data starts

// Function codes
#define FC_DISCOVER 0
#define FC_STREAM   1
#define FC_LED      2

// Message types
#define TP_DSC      1
#define TP_IMU      2
#define TP_MAG      3
#define TP_ADC      4

// =================== OTHER GLOBALS ==================== //
bool stream_status;
uint8_t led_color[3] = {0, 0, 0};
unsigned long ms_last, ms_accum;

// device info
struct devInfo_t {
  int chip_id;
  float gyro_cvt, acc_cvt, mag_cvt;
}
devInfo {
  0,
  (PI / 180.0f) * 1000.0f / 32768.0f,
  2.0f * 9.81f / 32768.0f,
  4912.0f / 32760.0f
};

uint8_t packet_num;
// ======================= SETUP ======================== //
void setup() {
  Serial.begin(115200);
  delay(1000);

  // init data
  Serial.println("Initializing");
  devInfo.chip_id = ESP.getChipId();

  // Setup LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Setup comm
  Wire.begin();
  Wire.setClock(400000);
  delay(1000);

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
  Serial.println("Starting wifi.");
  WiFi.begin();
  setColor(0, 0, 255);
  Serial.printf("WiFi connecting on %s", WiFi.SSID().c_str());
  int wifi_connect_counter = 0;
  int n_max_retry = 40;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    ++wifi_connect_counter;
    if (wifi_connect_counter > n_max_retry)
    { //------------------------------------------------------------- WiFi Failure Mode
      Serial.println("\nWiFi Failure mode:");
      bool blink_status = false;
      int login_step = 0;
      char pass[32], ssid[32];
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
            Serial.println("Connecting again with");
            Serial.println(pass);
            Serial.println(ssid);
            WiFi.persistent(true);
            WiFi.begin(ssid, pass);
            wifi_connect_counter = 0;
            setColor(0, 0, 255);
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
    delay(500);
  }
  Serial.println("\nWiFi connected succesfully!");
  delay(100);
  Serial.end();

  // Begin udp
  setColor(0, 255, 0);
  Udp.begin(localUdpPort);

  // Init timing
  ms_last = millis();
  ms_accum = 0;
  packet_num = 0;

  // Init LED state
  seconds_elapsed = 0;
  wake_status = true;
}

// ======================= LOOP ========================= //
void loop() {
  // check MPU data ready
  bool imu_data_ready = readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
  if (imu_data_ready) {
    // read accelerometer bytes
    uint8_t accBuffer[6];
    int16_t accel[3];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &accBuffer[0]);
    accel[0] = ((int16_t)accBuffer[0] << 8) | accBuffer[1];
    accel[1] = ((int16_t)accBuffer[2] << 8) | accBuffer[3];
    accel[2] = ((int16_t)accBuffer[4] << 8) | accBuffer[5];
    // read gyroscope bytes
    uint8_t gyroBuffer[6];
    int16_t gyro[3];
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &gyroBuffer[0]);
    gyro[0] = ((int16_t)gyroBuffer[0] << 8) | gyroBuffer[1];
    gyro[1] = ((int16_t)gyroBuffer[2] << 8) | gyroBuffer[3];
    gyro[2] = ((int16_t)gyroBuffer[4] << 8) | gyroBuffer[5];

    if (stream_status) {
      uint8_t head[4];
      head[MSG_FC] = FC_STREAM;
      head[MSG_TYPE] = TP_IMU;
      head[MSG_LEN] = 12;
      head[MSG_NUM] = ++packet_num;
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(head, 4);
      Udp.write((uint8_t*)accel, 6);
      Udp.write((uint8_t*)gyro, 6);
      Udp.endPacket();
    }
  }

  // Check magnetometer data ready
  bool mag_data_ready = readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01;
  if (mag_data_ready) {
    // ST2 register must be read ST2 at end of data acquisition
    uint8_t magBuffer[7];
    int16_t mag[3];
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &magBuffer[0]);
    // Check magnetic sensor overflow bit is not set
    bool mag_overflow = magBuffer[6] & 0x08;
    if (!mag_overflow) {
      // Transfer data from buffer
      mag[0] = ((int16_t)magBuffer[1] << 8) | magBuffer[0] ;
      mag[1] = ((int16_t)magBuffer[3] << 8) | magBuffer[2] ;
      mag[2] = ((int16_t)magBuffer[5] << 8) | magBuffer[4] ;
      if (stream_status) {
        uint8_t head[4];
        head[MSG_FC] = FC_STREAM;
        head[MSG_TYPE] = TP_MAG;
        head[MSG_LEN] = 6;
        head[MSG_NUM] = ++packet_num;
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(head, 4);
        Udp.write((uint8_t*)mag, 6);
        Udp.endPacket();
      }
    }
  }

  // check ADC data ready
  if (adcstate) {
    uint8_t adcBuffer[2] = {0};
    readBytes(ADS1115_ADDRESS, ADS_CONVERSION, 2, &adcBuffer[0]);
    adc_data[adc_active_channel] = ((int16_t)adcBuffer[0] << 8) | adcBuffer[1];
    ++adc_active_channel;
    if (adc_active_channel > 3) adc_active_channel = 0;
    writeByte2(ADS1115_ADDRESS, ADS_CONFIG, ch_cfg[adc_active_channel], cfg2);
    adcstate = false;

    // adc data streaming
    if (stream_status) {
      uint8_t head[4];
      head[MSG_FC] = FC_STREAM;
      head[MSG_TYPE] = TP_ADC;
      head[MSG_LEN] = 8;
      head[MSG_NUM] = ++packet_num;
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(head, 4);
      Udp.write((uint8_t*)adc_data, 8);
      Udp.endPacket();
    }
  }

  // Get timing
  unsigned long ms_now = millis();
  unsigned long elapsed = ms_now - ms_last;
  ms_last = ms_now;
  // accumulate elapsed time
  ms_accum += elapsed;

  // Count seconds
  if (ms_accum > 1000) {
    ++seconds_elapsed;
    ms_accum -= 1000;
  }

  // Sleep LED
  if (wake_status & (seconds_elapsed > led_timeout) ) {
    setColor(0, 0, 0);
    wake_status = false;
  }


  // Listen for incoming UDP packets
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    char buf[64] = {0};
    Udp.read(buf, sizeof(buf));

    // Run protocol
    switch (buf[MSG_FC]) {
      case FC_DISCOVER:
        buf[MSG_LEN] = sizeof(devInfo);
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(buf, 4);
        Udp.write((uint8_t*)&devInfo, sizeof(devInfo));
        Udp.endPacket();
        break;
      case FC_LED:
        led_color[0] = buf[MSG_DATA];
        led_color[1] = buf[MSG_DATA + 1];
        led_color[2] = buf[MSG_DATA + 2];
        led_timeout = buf[MSG_DATA + 3];
        break;
      case FC_STREAM:
        stream_status = !stream_status;
        break;
    }
    // wake led
    setColor(led_color[0], led_color[1], led_color[2]);
    seconds_elapsed = 0;
    wake_status = true;
  }

  // Stream data rate
  delay(20);
}

// ======================= READ / WRITE ========================= //
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
