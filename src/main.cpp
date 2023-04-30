#include <Arduino.h>
#include "Options.h"
#include "drive_base.h"
#include "imu.h"
#include "common/pid.h"
#include "data_stream.h"

// Create a start-up menu with 3 non-default options
Options options(4);
bool shouldCalibrateMotors;
bool enableFocStudio;
bool shouldCalibrateImu;

// Create a drive base object
DriveBase* drive_base = nullptr;

// Create IMU
Imu::Imu imu = Imu::Imu();

// Instantiate a PID controller for balancing
PIDController pid = PIDController(1.11f, 13.85f, 0.015f, 10000.0f, 20.0f);

// target velocities will be calculated as a percentage of the max velocity
constexpr float max_velocity_rad_per_sec = 25.0f;

// Create a data stream for the pitch and time
DataStream<float> pitch_data_stream = DataStream<float>("pitch", 1000);
DataStream<float> time_data_stream = DataStream<float>("time", 1000);

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

    drive_base = new DriveBase();

    int selected_option = options.getSelectedOption();
    shouldCalibrateMotors = selected_option == 0 || selected_option == 2;
    enableFocStudio = selected_option == 1 || selected_option == 2;
    shouldCalibrateImu = selected_option == 3;
    drive_base->init(shouldCalibrateMotors, enableFocStudio);
    imu.init(false);

    delay(1000);
    drive_base->enable();

    Serial.println("Letting IMU stabilize...");
    for (int i = 0; i < 1000; i++)
    {
        imu.loop();
    }
}

void loop()
{
    // Start time
    unsigned long start_time = micros();

    drive_base->loop();
    imu.loop();

    // Compute error
    // For balancing alone, we only need the pitch angle
    float error = imu.getPitch();
    bool time_success = time_data_stream.add_data_point(millis()/1000.0f);
    bool pitch_success = pitch_data_stream.add_data_point(error);

    float command = pid(error);

    drive_base->setTarget(command, command);

    if(!time_success)
    {
        pitch_data_stream.output_data_stream();
        time_data_stream.output_data_stream();
        pitch_data_stream.reset();
        time_data_stream.reset();
    }

    // Print loop time
    // Serial.print("Loop time: ");
    // Serial.print(micros() - start_time);


    // // Print out the current roll, pitch, and yaw
    // // RPY for the IMU aligns with the robot's frame of reference
    // // Pitch will be the forward/backward tilt
    // // Serial.print("Roll: ");
    // // Serial.print(imu.getRoll());
    // // Serial.print(" Yaw: ");
    // // Serial.println(imu.getYaw());
    // Serial.print(" Pitch: ");
    // Serial.println(imu.getPitch());
    // Serial.print(" Command: ");
    // Serial.print(command);
    // Serial.print(" Velocity: ");
    // Serial.println(drive_base -> getLeftVelocity());
}