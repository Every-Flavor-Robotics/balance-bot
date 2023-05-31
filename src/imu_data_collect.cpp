#include <Arduino.h>

#include "Options.h"
#include "common/pid.h"
#include "drive_base.h"
#include "imu.h"
#include "serial_data_stream.h"

// Create a start-up menu with 3 non-default options
Options options(4);
bool should_calibrate_motors;
bool enable_foc_studio;
bool should_calibrate_imu;

// // Create a drive base object
DriveBase* drive_base = nullptr;

// // Create IMU
Imu::Imu imu = Imu::Imu();

// // Instantiate a PID controller for balancing
PIDController pid = PIDController(1.11f, 13.85f, 0.015f, 10000.0f, 20.0f);

//////// IMU characterization data streams
// Raw gyro
DataStream::DataStream<float> raw_imu_gyro_x =
    DataStream::DataStream<float>("imu_gyro_x", 1000);
DataStream::DataStream<float> raw_imu_gyro_y =
    DataStream::DataStream<float>("imu_gyro_y", 2000);
DataStream::DataStream<float> raw_imu_gyro_z =
    DataStream::DataStream<float>("imu_gyro_z", 1000);

// Raw accel
DataStream::DataStream<float> raw_imu_accel_x =
    DataStream::DataStream<float>("imu_accel_x", 1000);
DataStream::DataStream<float> raw_imu_accel_y =
    DataStream::DataStream<float>("imu_accel_y", 1000);
DataStream::DataStream<float> raw_imu_accel_z =
    DataStream::DataStream<float>("imu_accel_z", 1000);

// Raw mag
DataStream::DataStream<float> raw_imu_mag_x =
    DataStream::DataStream<float>("imu_mag_x", 1000);
DataStream::DataStream<float> raw_imu_mag_y =
    DataStream::DataStream<float>("imu_mag_y", 1000);
DataStream::DataStream<float> raw_imu_mag_z =
    DataStream::DataStream<float>("imu_mag_z", 1000);

// Filtered streams
DataStream::DataStream<float> imu_roll =
    DataStream::DataStream<float>("imu_roll", 1000);
DataStream::DataStream<float> imu_pitch =
    DataStream::DataStream<float>("imu_pitch", 2000);
DataStream::DataStream<float> imu_yaw =
    DataStream::DataStream<float>("imu_yaw", 1000);

// time
DataStream::DataStream<float> time_data =
    DataStream::DataStream<float>("time", 2000);

int num_loops = 0;

void setup()
{
  // start serial
  Serial.begin(115200);

  // Add options and run menu before setting up motor
  options.addOption("Recalibrate motor");
  options.addOption("FOCStudio");
  options.addOption("Recalibrate motor + FOCStudio");
  options.addOption("Calibrate IMU");
  options.run();

  // Determine if imu and/or motors should be calibrated based on user input
  int selected_option = options.getSelectedOption();
  should_calibrate_motors = selected_option == 0 || selected_option == 2;
  enable_foc_studio = selected_option == 1 || selected_option == 2;
  should_calibrate_imu = selected_option == 3;

  // Init IMU and drive base
  drive_base = new DriveBase();
  drive_base->init(should_calibrate_motors, enable_foc_studio);
  imu.init(false);

  // Wait for everything to start up
  delay(1000);

  // Allow filter to stabilize
  Serial.println("Letting IMU stabilize... Hold IMU vertical");
  float pitch = 0.0f;
  while (pitch > 0.1)
  {
    pitch = imu.get_pitch();
    delay(10);
  }

  // Enable drive base and begin data collection
  drive_base->enable();
  Serial.println(millis() / 1000.0f);
  Serial.println("Starting data collection!");
  delay(500);
}

void loop()
{
  drive_base->loop();
  imu.loop();

  // Serial.println(imu.getPitch());
  // Compute error
  // For balancing alone, we only need the pitch angle

  float command = 0.0f;

  // log data
  time_data.add_data_point((float)millis() / 1000.0f);
  raw_imu_accel_x.add_data_point(imu.get_raw_accel_x());
  raw_imu_accel_y.add_data_point(imu.get_raw_accel_y());
  raw_imu_accel_z.add_data_point(imu.get_raw_accel_z());

  raw_imu_gyro_x.add_data_point(imu.get_raw_gyro_x());
  raw_imu_gyro_y.add_data_point(imu.get_raw_gyro_y());
  raw_imu_gyro_z.add_data_point(imu.get_raw_gyro_z());

  raw_imu_mag_x.add_data_point(imu.get_raw_mag_x());
  raw_imu_mag_y.add_data_point(imu.get_raw_mag_y());
  raw_imu_mag_z.add_data_point(imu.get_raw_mag_z());

  imu_roll.add_data_point(imu.get_roll());
  imu_yaw.add_data_point(imu.get_yaw());
  bool log_success = imu_pitch.add_data_point(imu.get_pitch());
  // // Store success to know when log is full

  // Set target to current pitch
  drive_base->set_target(command, command);

  // // Data full, flush to computer
  if (!log_success)
  {
    // Disable drive_base
    drive_base->disable();

    Serial.println("Data stream full, flushing to computer");
    Serial.println(millis() / 1000.0f);

    // Set postfix to numLoops
    const char* postfix = "";
    delay(100);

    raw_imu_accel_x.output_data_stream(postfix);
    raw_imu_accel_y.output_data_stream(postfix);
    raw_imu_accel_z.output_data_stream(postfix);

    raw_imu_gyro_x.output_data_stream(postfix);
    raw_imu_gyro_y.output_data_stream(postfix);
    raw_imu_gyro_z.output_data_stream(postfix);

    raw_imu_mag_x.output_data_stream(postfix);
    raw_imu_mag_y.output_data_stream(postfix);
    raw_imu_mag_z.output_data_stream(postfix);

    imu_roll.output_data_stream(postfix);
    imu_pitch.output_data_stream(postfix);
    imu_yaw.output_data_stream(postfix);
    time_data.output_data_stream(postfix);

    // Reset data streams
    raw_imu_accel_x.reset();
    raw_imu_accel_y.reset();
    raw_imu_accel_z.reset();

    raw_imu_gyro_x.reset();
    raw_imu_gyro_y.reset();
    raw_imu_gyro_z.reset();

    raw_imu_mag_x.reset();
    raw_imu_mag_y.reset();
    raw_imu_mag_z.reset();

    imu_roll.reset();
    imu_pitch.reset();
    imu_yaw.reset();

    time_data.reset();

    num_loops++;

    // Loop until user presses reset
    while (true)
    {
      // do nothing
    }
  }

  if (num_loops == 1)
  {
    // Close the data logging session, name file imu_stream
    DataStream::close("imu_stream");
  }
}