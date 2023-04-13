#include <imu.h>


namespace Imu {

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

void serial_print_motioncal(sensors_event_t &accel_event, sensors_event_t &gyro_event, sensors_event_t &mag_event)
{
    Serial.print("Raw:");
    Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
    Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
    Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
    Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
    Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
    Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");
    // unified data
    Serial.print("Uni:");
    Serial.print(accel_event.acceleration.x); Serial.print(",");
    Serial.print(accel_event.acceleration.y); Serial.print(",");
    Serial.print(accel_event.acceleration.z); Serial.print(",");
    Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
    Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
    Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
    Serial.print(mag_event.magnetic.x); Serial.print(",");
    Serial.print(mag_event.magnetic.y); Serial.print(",");
    Serial.print(mag_event.magnetic.z); Serial.println("");
}

// Return char array with key name for calibration data
void get_key(const char* name, char* key)
{
    sprintf(key, "imu_%s", name);
}

void writeCalibrationData(imu_calibration_data_t &data, const char* name)
{

    // Open preferences with namespace "calibration"
    Preferences preferences;
    preferences.begin("imu", false);

    // Get key name
    char key[32];
    get_key(name, key);

    // Write data to preferences
    preferences.putBytes(key, &data.raw, sizeof(data));

    // Close preferences
    preferences.end();
}

imu_calibration_data_t readCalibrationData(const char* name)
{
    imu_calibration_data_t data;

    // Open preferences with namespace "calibration"
    Preferences preferences;
    preferences.begin("imu", false);

    // Get key name
    char key[32];
    get_key(name, key);

    // Read data from preferences
    preferences.getBytes(key, &data.raw, sizeof(data));

    // Close preferences
    preferences.end();

    return data;
}



Imu::Imu()
{
    // Instantiate IMU objects
    bool lsm6ds_success = lsm6ds.begin_I2C();
    bool lis3mdl_success = lis3mdl.begin_I2C();
}

void Imu::init(bool shouldCalibrate)
{
    // TODO: figure out how to calibrate the IMU



    // Configure IMU rates and ranges
    // Set IMU range
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);

    // Set IMU data rates
    lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

    // Set up magnetometer
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    lis3mdl.setIntThreshold(500);
    lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

    // IMU configured
    delay(1000);

    // Calibrate IMU
    if (shouldCalibrate)
    {
        calibrate();
    }
    else
    {
        // Read calibration data from flash
        imu_calibration_data_t calibrationData = readCalibrationData("A");
    }
}


// Default initializer
void Imu::init()
{
    init(false);
}

void Imu::read()
{
    //  /* Get new normalized sensor events */
    lsm6ds.getEvent(&accel_event, &gyro_event, &temp);
    lis3mdl.getEvent(&mag_event);
}

void Imu::calibrate()
{
    const int LED_PIN = 2;
    pinMode(LED_PIN, OUTPUT);

    bool calibrationReceived = false;

    // Create calibration object to populate
    imu_calibration_data_t calibrationData;

    // Loop until we receive calibration data from motioncal
    size_t loopcount = 0;
    while(!calibrationReceived)
    {
        // Read data from IMU and print in motioncal format
        read();
        serial_print_motioncal(accel_event, gyro_event, mag_event);

        // Update calibration status
        calibrationReceived = receiveCalibration();
        loopcount++;
    }

    // Save magnetic calibration data to struct
    calibrationData.mag_soft_iron[0] = offsets[10];
    calibrationData.mag_soft_iron[1] = offsets[13];
    calibrationData.mag_soft_iron[2] = offsets[14];
    calibrationData.mag_soft_iron[3] = offsets[13];
    calibrationData.mag_soft_iron[4] = offsets[11];
    calibrationData.mag_soft_iron[5] = offsets[15];
    calibrationData.mag_soft_iron[6] = offsets[14];
    calibrationData.mag_soft_iron[7] = offsets[15];
    calibrationData.mag_soft_iron[8] = offsets[12];

    calibrationData.mag_hard_iron[0] = offsets[6];
    calibrationData.mag_hard_iron[1] = offsets[7];
    calibrationData.mag_hard_iron[2] = offsets[8];

    // Now we need to calibrate the gyro and accel
    // Blink onboard led three times to indicate calibration starting
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }

    delay(1000);

    // Calibrate the gyro and accel
    // Loop 1000 times

    // Create 6 floats to store the running sum of the gyro and accel data
    float gyroXSum = 0;
    float gyroYSum = 0;
    float gyroZSum = 0;
    float accelXSum = 0;
    float accelYSum = 0;
    float accelZSum = 0;


    for (int i = 0; i < 1000; i++)
    {
        // Read data from IMU and print in motioncal format
        read();

        // Add gyro and accel data to running sum
        gyroXSum += gyro_event.gyro.x;
        gyroYSum += gyro_event.gyro.y;
        gyroZSum += gyro_event.gyro.z;

        accelXSum += accel_event.acceleration.x;
        accelYSum += accel_event.acceleration.y;
        accelZSum += accel_event.acceleration.z;

        // Blink onboard led to indicate calibration in progress
        digitalWrite(LED_PIN, HIGH);
        delay(1);
        digitalWrite(LED_PIN, LOW);
        delay(1);
    }

    // Calculate the average gyro and accel data
    calibrationData.gyro_bias[0] = gyroXSum / 1000;
    calibrationData.gyro_bias[1] = gyroYSum / 1000;
    calibrationData.gyro_bias[2] = gyroZSum / 1000;

    calibrationData.accel_bias[0] = accelXSum / 1000;
    calibrationData.accel_bias[1] = accelYSum / 1000;
    calibrationData.accel_bias[2] = accelZSum / 1000;

    // Save calibration data using preferences
    writeCalibrationData(calibrationData, "A");
}


bool Imu::receiveCalibration() {
  uint16_t crc;
  byte b, i;

  while (Serial.available()) {
    b = Serial.read();
    if (calcount == 0 && b != 117) {
      // first byte must be 117
      return false;
    }
    if (calcount == 1 && b != 84) {
      // second byte must be 84
      calcount = 0;
      return false;
    }
    // store this byte
    caldata[calcount++] = b;
    if (calcount < 68) {
      // full calibration message is 68 bytes
      return false;
    }
    // verify the crc16 check
    crc = 0xFFFF;
    for (i=0; i < 68; i++) {
      crc = crc16_update(crc, caldata[i]);
    }
    if (crc == 0) {
      // data looks good, use it
      memcpy(offsets, caldata+2, 16*4);

      calcount = 0;
      return true;
    }
    // look for the 117,84 in the data, before discarding
    for (i=2; i < 67; i++) {
      if (caldata[i] == 117 && caldata[i+1] == 84) {
        // found possible start within data
        calcount = 68 - i;
        memmove(caldata, caldata + i, calcount);
        return false;
      }
    }
    // look for 117 in last byte
    if (caldata[67] == 117) {
      caldata[0] = 117;
      calcount = 1;
    } else {
      calcount = 0;
    }
  }
    return false;
}

} // namespace Imu