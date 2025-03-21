#include "ICM_20948.h"
#include <math.h>
#include <SPI.h>

#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

static const int PIN_MOSI_ROLL = 9;
static const int PIN_MISO_ROLL = 8;
static const int PIN_SCK_ROLL = 7;

static const int PIN_MOSI_PITCH = 2;
static const int PIN_MISO_PITCH = 3;
static const int PIN_SCK_PITCH = 4;


SPIClass SPI2(HSPI); // SPI is roll, SPI2 is pitch

void setup() {
  Serial.begin(115200);  // Start the serial console


  WIRE_PORT.begin(5, 6);
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {

    myICM.begin(WIRE_PORT, AD0_VAL);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());

    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Trying again..."));
      delay(500);
    } else {
      initialized = true;
    }
  }

  Serial.println(F("Device connected!"));

  bool success = true;

  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);  // Set to the maximum


  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
    Serial.println(F("DMP enabled!"));
  } else {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ;
  }

  SPI.begin(PIN_MOSI_ROLL, PIN_MISO_ROLL, PIN_SCK_ROLL);
  SPI2.begin(PIN_MOSI_PITCH, PIN_MISO_PITCH, PIN_SCK_PITCH);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI2.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat9) > 0)  // We have asked for orientation data so we should receive Quat9
    {
      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double yaw, pitch, roll;
      quaternionToEuler(q0, q1, q2, q3, yaw, pitch, roll);

      yaw = yaw * 180.0 / M_PI;
      pitch = pitch * 180.0 / M_PI;
      roll = roll * 180.0 / M_PI;

      // roll
      union {
        float value;
        byte bytes[4];
      } floatData;
      floatData.value = roll;

      Serial.print("roll: ");
      Serial.print(floatData.value, 2);

      SPI.transfer(0xAA);  // Start Transmission Signal

      for (int i = 0; i < 4; i++) {
        SPI.transfer(floatData.bytes[i]);
      }

      // pitch
      union {
        float value;
        byte bytes[4];
      } floatData2;
      floatData2.value = pitch;

      Serial.print(", pitch: ");
      Serial.println(floatData2.value, 2);

      SPI2.transfer(0xAA);  // Start Transmission Signal

      for (int i = 0; i < 4; i++) {
        SPI2.transfer(floatData2.bytes[i]);
      }
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)  // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}


void quaternionToEuler(double q0, double q1, double q2, double q3,
                       double &yaw, double &pitch, double &roll) {
  double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
  double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
  yaw = atan2(siny_cosp, cosy_cosp);

  double sinp = 2.0 * (q0 * q2 - q3 * q1);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
  else
    pitch = asin(sinp);

  double sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
  double cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
  roll = atan2(sinr_cosp, cosr_cosp);
}