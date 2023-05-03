// #include <Arduino.h>
// #include "Options.h"
// #include "drive_base.h"
// #include "imu.h"
// #include "common/pid.h"
// #include "data_stream.h"

// // Create a start-up menu with 3 non-default options
// Options options(4);
// bool shouldCalibrateMotors;
// bool enableFocStudio;
// bool shouldCalibrateImu;

// // // Create a drive base object
// DriveBase* drive_base = nullptr;

// // // Create IMU
// Imu::Imu imu = Imu::Imu();

// // // Instantiate a PID controller for balancing
// PIDController pid = PIDController(1.11f, 13.85f, 0.015f, 10000.0f, 20.0f);

// // IMU characterization data streams
// DataStream<float> raw_imu_gyro_x = DataStream<float>("imu_gyro_x", 1000);
// DataStream<float> raw_imu_gyro_y = DataStream<float>("imu_gyro_y", 1000);
// DataStream<float> raw_imu_gyro_z = DataStream<float>("imu_gyro_z", 1000);

// DataStream<float> raw_imu_accel_x = DataStream<float>("imu_accel_x", 1000);
// DataStream<float> raw_imu_accel_y = DataStream<float>("imu_accel_y", 1000);
// DataStream<float> raw_imu_accel_z = DataStream<float>("imu_accel_z", 1000);

// DataStream<float> raw_imu_mag_x = DataStream<float>("imu_mag_x", 1000);
// DataStream<float> raw_imu_mag_y = DataStream<float>("imu_mag_y", 1000);
// DataStream<float> raw_imu_mag_z = DataStream<float>("imu_mag_z", 1000);

// DataStream<float> imu_roll  = DataStream<float>("imu_roll", 1000);
// DataStream<float> imu_pitch = DataStream<float>("imu_pitch", 1000);
// DataStream<float> imu_yaw   = DataStream<float>("imu_yaw", 1000);

// DataStream<float> time_data = DataStream<float>("time", 1000);

// int numLoops = 0;

// void setup_helper()
// {
//     // start serial
//     Serial.begin(115200);

//     // Add options and run menu before setting up motor
//     options.addOption("Recalibrate motor");
//     options.addOption("FOCStudio");
//     options.addOption("Recalibrate motor + FOCStudio");
//     options.addOption("Calibrate IMU");
//     options.run();

//     drive_base = new DriveBase();

//     int selected_option = options.getSelectedOption();
//     shouldCalibrateMotors = selected_option == 0 || selected_option == 2;
//     enableFocStudio = selected_option == 1 || selected_option == 2;
//     shouldCalibrateImu = selected_option == 3;
//     drive_base->init(shouldCalibrateMotors, enableFocStudio);
//     imu.init(false);

//     delay(1000);
//     drive_base->enable();

//     Serial.println("Letting IMU stabilize...");
//     for (int i = 0; i < 1000; i++)
//     {
//         imu.loop();
//     }

//     Serial.println("Starting data collection!");
// }

// void loop_helper()
// {

//     drive_base->loop();
//     imu.loop();
//     // Compute error
//     // For balancing alone, we only need the pitch angle

//     float command = 0.0f;

//     // log data
//     time_data.add_data_point((float)millis() / 1000.0f);
//     raw_imu_accel_x.add_data_point(imu.getRawAccelX());
//     raw_imu_accel_y.add_data_point(imu.getRawAccelY());
//     raw_imu_accel_z.add_data_point(imu.getRawAccelZ());

//     raw_imu_gyro_x.add_data_point(imu.getRawGyroX());
//     raw_imu_gyro_y.add_data_point(imu.getRawGyroY());
//     raw_imu_gyro_z.add_data_point(imu.getRawGyroZ());

//     raw_imu_mag_x.add_data_point(imu.getRawMagX());
//     raw_imu_mag_y.add_data_point(imu.getRawMagY());
//     raw_imu_mag_z.add_data_point(imu.getRawMagZ());

//     imu_roll.add_data_point(imu.getRoll());
//     imu_pitch.add_data_point(imu.getPitch());
//     // Store success to know when log is full
//     bool logSuccess = imu_yaw.add_data_point(imu.getYaw());

//     drive_base->setTarget(command, command);

//     // Data full, flush to computer
//     if(!logSuccess)
//     {
//         // pitch_data_stream.output_data_stream();
//         // time_data_stream.output_data_stream();
//         // command_velocity_data_stream.output_data_stream();
//         // velocity_data_stream.output_data_stream();
//         // voltage_data_stream.output_data_stream();

//         //         pitch_data_stream.reset();
//         // time_data_stream.reset();
//         // command_velocity_data_stream.reset();
//         // velocity_data_stream.reset();
//         // voltage_data_stream.reset();

//         // Serial.println("Data stream full, flushing to computer");

//         // Set postfix to numLoops
//         const char* postfix = String(numLoops).c_str();

//         raw_imu_accel_x.output_data_stream(postfix);
//         raw_imu_accel_y.output_data_stream(postfix);
//         raw_imu_accel_z.output_data_stream(postfix);

//         raw_imu_gyro_x.output_data_stream(postfix);
//         raw_imu_gyro_y.output_data_stream(postfix);
//         raw_imu_gyro_z.output_data_stream(postfix);

//         raw_imu_mag_x.output_data_stream(postfix);
//         raw_imu_mag_y.output_data_stream(postfix);
//         raw_imu_mag_z.output_data_stream(postfix);

//         imu_roll.output_data_stream(postfix);
//         imu_pitch.output_data_stream(postfix);
//         imu_yaw.output_data_stream(postfix);

//         time_data.output_data_stream(postfix);

//         // Reset data streams
//         raw_imu_accel_x.reset();
//         raw_imu_accel_y.reset();
//         raw_imu_accel_z.reset();

//         raw_imu_gyro_x.reset();
//         raw_imu_gyro_y.reset();
//         raw_imu_gyro_z.reset();

//         raw_imu_mag_x.reset();
//         raw_imu_mag_y.reset();
//         raw_imu_mag_z.reset();

//         imu_roll.reset();
//         imu_pitch.reset();
//         imu_yaw.reset();

//         time_data.reset();

//         numLoops++;
//     }

//     if(numLoops == 2)
//     {
//         close("imu_stream");
//     }
// }