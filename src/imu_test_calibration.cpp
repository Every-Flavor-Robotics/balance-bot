#include <Arduino.h>
#include "Options.h"
#include "drive_base.h"



// Create IMU sensor devices
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;

// Create a start-up menu with 3 non-default options
Options options(3);
bool shouldCalibrate;
bool enableFocStudio;

float offsets[16];


byte caldata[68]; // buffer to receive magnetic calibration data
byte calcount=0;

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

// Create a drive base object
// DriveBase drive_base = DriveBase();

// target velocities will be calculated as a percentage of the max velocity
constexpr float max_velocity_rad_per_sec = 25.0f;

void setup()
{
    // start serial
    Serial.begin(115200);



}


int loopcount = 0;
void loop()
{

//   // occasionally print calibration
//   if (loopcount == 50 || loopcount > 100) {
//     Serial.print("Cal1:");
//     for (int i=0; i<3; i++) {
//       Serial.print(offsets[i], 3);
//       Serial.print(",");
//     }
//     for (int i=3; i<6; i++) {
//       Serial.print(offsets[i], 3);
//       Serial.print(",");
//     }
//     for (int i=6; i<9; i++) {
//       Serial.print(offsets[i], 3);
//       Serial.print(",");
//     }
//     Serial.println(offsets[9], 3);
//     loopcount++;
//   }
//   if (loopcount >= 100) {
//     Serial.print("Cal2:");
//     Serial.print(offsets[10], 3); Serial.print(",");
//     Serial.print(offsets[13], 3); Serial.print(",");
//     Serial.print(offsets[14], 3); Serial.print(",");
//     Serial.print(offsets[13], 3); Serial.print(",");
//     Serial.print(offsets[11], 3); Serial.print(",");
//     Serial.print(offsets[15], 3); Serial.print(",");
//     Serial.print(offsets[14], 3); Serial.print(",");
//     Serial.print(offsets[15], 3); Serial.print(",");
//     Serial.print(offsets[12], 3);
//     Serial.println();
//     loopcount = 0;
//   }

  delay(10);
}
