// #include "Arduino.h"
// #include "Options.h"
// #include "common/lowpass_filter.h"
// #include "common/pid.h"
// #include "drive_base.h"
// #include "imu.h"
// #include "serial_data_stream.h"

// // Create a start-up menu with 3 non-default options
// Options options(4);
// bool shouldCalibrateMotors;
// bool enableFocStudio;
// bool shouldCalibrateImu;

// // Create a drive base object
// DriveBase* drive_base = nullptr;

// // Create IMU
// Imu::Imu imu = Imu::Imu();

// float last_pitch = 0.0f;
// float last_pitch_rate = 0.0f;
// float last_time = 0.0f;
// float last_dt = 0.0f;
// float last_wheel_velocity = 0.0f;

// float k_0 = 0.03535534f;
// float k_1 = 3.11469196f;
// float k_2 = 23.83802148f;
// float k_3 = 13.94165592f;

// float zero_wheel = 0.0f;
// float angle_offset = -0.029824;
// float target_freq = 250.0f;

// // We need to log the following:
// // - time
// // - pitch
// // - pitch rate
// // - pitch accel
// // - wheel angle
// // - wheel rate
// // - wheel accel
// // - torque command

// DataStream::DataStream<float> imu_pitch =
//     DataStream::DataStream<float>("imu_pitch", 1000);
// DataStream::DataStream<float> imu_pitch_rate =
//     DataStream::DataStream<float>("imu_pitch_rate", 1000);
// DataStream::DataStream<float> imu_pitch_rate_filter =
//     DataStream::DataStream<float>("imu_pitch_rate_filter", 2000);
// DataStream::DataStream<float> imu_pitch_rate_gyro =
//     DataStream::DataStream<float>("imu_pitch_rate_gyro", 2000);
// DataStream::DataStream<float> imu_pitch_accel =
//     DataStream::DataStream<float>("imu_pitch_accel", 1000);

// DataStream::DataStream<float> wheel_angle =
//     DataStream::DataStream<float>("wheel_angle", 1000);
// DataStream::DataStream<float> wheel_rate =
//     DataStream::DataStream<float>("wheel_rate", 1000);
// DataStream::DataStream<float> wheel_accel =
//     DataStream::DataStream<float>("wheel_accel", 1000);

// DataStream::DataStream<float> torque_command =
//     DataStream::DataStream<float>("torque_command", 1000);
// DataStream::DataStream<float> time_data =
//     DataStream::DataStream<float>("time", 1000);
// DataStream::DataStream<float> controller =
//     DataStream::DataStream<float>("k", 4);

// int i = 0;

// void setup()
// {
//   // start serial
//   Serial.begin(115200);

//   controller.add_data_point(k_0);
//   controller.add_data_point(k_1);
//   controller.add_data_point(k_2);
//   controller.add_data_point(k_3);

//   // Add options and run menu before setting up motor
//   options.addOption("Recalibrate motor");
//   options.addOption("FOCStudio");
//   options.addOption("Recalibrate motor + FOCStudio");
//   options.addOption("Calibrate IMU");
//   options.run();

//   int selected_option = options.getSelectedOption();
//   shouldCalibrateMotors = selected_option == 0 || selected_option == 2;
//   enableFocStudio = selected_option == 1 || selected_option == 2;
//   shouldCalibrateImu = selected_option == 3;

//   drive_base = new DriveBase();
//   drive_base->init(shouldCalibrateMotors, enableFocStudio);
//   imu.init(false);

//   delay(1000);
//   drive_base->enable();

//   Serial.println("Letting IMU stabilize...");
//   int stable_count = 0;
//   while (stable_count < 50)
//   {
//     imu.loop();
//     // pitch_rate_filter(imu.getRawGyroY() * 0.0174533f);
//     if (imu.getPitch() > 0.02f || imu.getPitch() < -0.02f)
//     {
//       stable_count = 0;
//     }
//     else
//     {
//       stable_count++;
//     }
//   }

//   last_pitch = imu.getPitch();
//   last_time = (float)millis() / 1000.0f;
//   Serial.println("Starting loop...");
//   zero_wheel = drive_base->getLeftPosition() - last_pitch;

//   delay(50);
// }

// void loop()
// {
//   // Start time
//   unsigned long start_time = micros();

//   imu.loop();

//   //////// Apply controller

//   // Generate state
//   // theta (wheel angle)
//   // theta_dot
//   // phi (pitch)
//   // phi_dot

//   float time = (float)millis() / 1000.0f;
//   // float dt = time - last_time;
//   float pitch = (imu.getPitch());
//   // Convert deg/s to rad/s
//   float pitchRate =
//       imu.getRawGyroY() *
//       0.0174533f;  // pitch_rate_filter(imu.getRawGyroY() * 0.0174533f);

//   float wheel_position = drive_base->getLeftPosition() - zero_wheel;
//   float wheel_velocity = drive_base->getLeftVelocity();

//   // Serial.println(wheel_velocity);

//   // u = -kx
//   float command =
//       -(k_0 * (wheel_position + pitch) + k_1 * (wheel_velocity + pitchRate) +
//         k_2 * pitch + k_3 * pitchRate);

//   drive_base->setTarget(command, command);
//   drive_base->loop();

//   // Wait for startup transients to die out before logging
//   if (i > 1000)
//   {
//     //
//     // float ddt = (dt + last_dt)/2.0f;

//     // // Compute pitch accel
//     // float phi_ddot = (dpdt - last_pitch_rate)/ddt;

//     // // Compute wheel accel
//     // float theta_ddot = (wheel_velocity - last_wheel_velocity)/ddt;

//     imu_pitch.add_data_point(pitch);
//     imu_pitch_rate.add_data_point(pitchRate);
//     imu_pitch_rate_filter.add_data_point(imu.getPitchRate());
//     imu_pitch_rate_gyro.add_data_point(imu.getRawGyroY());
//     // imu_pitch_accel.add_data_point(phi_ddot);

//     wheel_angle.add_data_point(wheel_position);
//     wheel_rate.add_data_point(wheel_velocity);
//     // wheel_accel.add_data_point(theta_ddot);

//     torque_command.add_data_point(command);
//     bool logSuccess = time_data.add_data_point(time);

//     if (!logSuccess)
//     {
//       // Flush to serial
//       imu_pitch.output_data_stream("");
//       imu_pitch_rate.output_data_stream("");
//       imu_pitch_rate_filter.output_data_stream("");
//       imu_pitch_rate_gyro.output_data_stream("");
//       // imu_pitch_accel.output_data_stream("");

//       wheel_angle.output_data_stream("");
//       wheel_rate.output_data_stream("");
//       // wheel_accel.output_data_stream("");

//       torque_command.output_data_stream("");
//       time_data.output_data_stream("");
//       controller.output_data_stream("");

//       DataStream::close("closed_loop");
//     }
//   }

//   // Sleep to maintain target frequency
//   unsigned long end_time = micros();
//   unsigned long elapsed_time = end_time - start_time;
//   unsigned long target_time = (unsigned long)(1000000.0f / target_freq);
//   if (elapsed_time < target_time)
//   {
//     delayMicroseconds(target_time - elapsed_time);
//   }

//   last_time = time;
//   i++;
// }
