#include <Arduino.h>
#include "Options.h"
#include "drive_base.h"
#include "imu.h"
#include "common/pid.h"

// Create a start-up menu with 3 non-default options
Options options(4);
bool shouldCalibrateMotors;
bool enableFocStudio;
bool shouldCalibrateImu;

// Create a drive base object
DriveBase drive_base = DriveBase();

// Create IMU
Imu::Imu imu = Imu::Imu();

// Instantiate a PID controller for balancing
PIDController pid = PIDController(0.3f, 0.001f, 0.0f, 5.0f, 7.0f);

// target velocities will be calculated as a percentage of the max velocity
constexpr float max_velocity_rad_per_sec = 25.0f;

void setup()
{
    // start serial
    Serial.begin(115200);

    // Add options and run menu before setting up motor
    options.addOption("Recalibrate motor");
    options.addOption("FOCStudio");
    options.addOption("Recalibrate motor + FOCStudio");
    options.run();
    int selected_option = options.getSelectedOption();
    shouldCalibrateMotors = selected_option == 0 || selected_option == 2;
    enableFocStudio = selected_option == 1 || selected_option == 2;
    shouldCalibrateImu = selected_option == 3;

    drive_base.init(shouldCalibrateMotors, enableFocStudio);
    imu.init(shouldCalibrateImu);

    delay(1000);
}

void loop()
{
    drive_base.loop();
    imu.loop();

    // Compute error
    // For balancing alone, we only need the pitch angle
    float error = imu.getPitch();

    float command = pid.operator(error);

    drive_base.setTarget(command, command);


    // Print out the current roll, pitch, and yaw
    // RPY for the IMU aligns with the robot's frame of reference
    // Pitch will be the forward/backward tilt
    // Serial.print("Roll: ");
    // Serial.print(imu.getRoll());
    Serial.print(" Pitch: ");
    Serial.print(imu.getPitch());

    Serial.print(" Command: ");
    Serial.print(command);
    // Serial.print(" Yaw: ");
    // Serial.println(imu.getYaw());





}