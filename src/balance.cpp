// #include "Options.h"
// #include "drive_base.h"
// #include "imu.h"
// #include "common/pid.h"




// // Create a start-up menu with 3 non-default options
// Options options(4);
// bool shouldCalibrateMotors;
// bool enableFocStudio;
// bool shouldCalibrateImu;

// // Create a drive base object
// DriveBase* drive_base = nullptr;

// // Create IMU
// Imu::Imu imu = Imu::Imu();

// // Instantiate a PID controller for balancing
// // PIDController pid = PIDController(1.11f, 13.85f, 0.015f, 10000.0f, 20.0f);

// float last_pitch = 0.0f;
// float last_time = 0.0f;

// float k_0 = 0.4472136f;
// float k_1 = 0.59392093f;
// float k_2 = 10.88024376f;
// float k_3 = 3.16343493f;

// float zero_wheel = 0.0f;


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

//     last_pitch = imu.getPitch() * PI / 180.0f;
//     last_time = (float)millis()/1000.0f;
//     Serial.println("Starting loop...");

//     delay(50);
//     zero_wheel = drive_base->getLeftPosition();


// }

// void loop_helper()
// {
//     // Start time
//     // unsigned long start_time = micros();

//     drive_base->loop();
//     imu.loop();

//     // Compute error
//     // For balancing alone, we only need the pitch angle, convert to radians
//     float pitch = (imu.getPitch()) * PI / 180.0f;

//     float time = (float)millis()/1000.0f;
//     float dt = time - last_time;
//     float dp = pitch - last_pitch;
//     float dpdt = dp/dt;

//     float wheel_position = drive_base->getLeftPosition();
//     float wheel_velocity = drive_base->getLeftVelocity();


//     float command = (k_0*wheel_position + k_1*wheel_velocity + k_2*pitch + k_3*dpdt);

//     drive_base->setTarget(command, command);


//     // Print control variables
//     // Serial.print("Wheel Position: ");
//     // Serial.print(wheel_position);
//     // Serial.print(" Wheel Velocity: ");
//     // Serial.print(wheel_velocity);
//     // Serial.print(" Pitch: ");
//     // Serial.print(pitch);
//     // Serial.print(" dpdt: ");
//     // Serial.print(dpdt);
//     // Serial.print(" Command: ");
//     // Serial.println(command);


//     // Print loop time
//     // Serial.print("Loop time: ");
//     // Serial.print(micros() - start_time);


//     // // Print out the current roll, pitch, and yaw
//     // // RPY for the IMU aligns with the robot's frame of reference
//     // // Pitch will be the forward/backward tilt
//     // // Serial.print("Roll: ");
//     // // Serial.print(imu.getRoll());
//     // // Serial.print(" Yaw: ");
//     // // Serial.println(imu.getYaw());
//     // Serial.print(" Pitch: ");
//     // Serial.println(imu.getPitch());
//     // Serial.print(" Command: ");
//     // Serial.print(command);
//     // Serial.print(" Velocity: ");
//     // Serial.println(drive_base -> getLeftVelocity());
// }