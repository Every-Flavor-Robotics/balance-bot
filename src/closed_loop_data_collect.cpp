#include "Arduino.h"
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


float last_pitch = 0.0f;
float last_pitch_rate = 0.0f;
float last_time = 0.0f;
float last_dt = 0.0f;
float last_wheel_velocity = 0.0f;


float k_0 = 0.4472136f;
float k_1 = 0.59392093f;
float k_2 = 10.88024376f;
float k_3 = 3.16343493f;

float zero_wheel = 0.0f;


// We need to log the following:
// - time
// - pitch
// - pitch rate
// - pitch accel
// - wheel angle
// - wheel rate
// - wheel accel
// - torque command

DataStream<float> imu_pitch = DataStream<float>("imu_pitch", 1000);
DataStream<float> imu_pitch_rate = DataStream<float>("imu_pitch_rate", 1000);
DataStream<float> imu_pitch_accel = DataStream<float>("imu_pitch_accel", 1000);

DataStream<float> wheel_angle = DataStream<float>("wheel_angle", 1000);
DataStream<float> wheel_rate = DataStream<float>("wheel_rate", 1000);
DataStream<float> wheel_accel = DataStream<float>("wheel_accel", 1000);

DataStream<float> torque_command = DataStream<float>("torque_command", 1000);
DataStream<float> time_data = DataStream<float>("time", 1000);


int i = 0;

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

    last_pitch = imu.getPitch();
    last_time = (float)millis()/1000.0f;
    Serial.println("Starting loop...");

    delay(50);
    zero_wheel = drive_base->getLeftPosition();
}

void loop()
{
    // Start time
    // unsigned long start_time = micros();

    drive_base->loop();
    imu.loop();


    //////// Apply controller

    // Generate state
    // theta (wheel angle)
    // theta_dot
    // phi (pitch)
    // phi_dot

    float pitch = (imu.getPitch());

    // Compute pitch rate
    float time = (float)millis()/1000.0f;
    float dt = time - last_time;
    float dp = pitch - last_pitch;
    float dpdt = dp/dt;

    float wheel_position = drive_base->getLeftPosition();
    float wheel_velocity = drive_base->getLeftVelocity();

    // u = -kx
    float command = -(k_0*wheel_position + k_1*wheel_velocity + k_2*pitch + k_3*dpdt);

    drive_base->setTarget(command, command);

    // Wait for startup transients to die out before logging
    if(i > 1000)
    {
        // Get total dt
        float ddt = dt + last_dt;

        // Compute pitch accel
        float phi_ddot = (dpdt - last_pitch_rate)/ddt;

        // Compute wheel accel
        float theta_ddot = (wheel_velocity - last_wheel_velocity)/ddt;

        imu_pitch.add_data_point(pitch);
        imu_pitch_rate.add_data_point(dpdt);
        imu_pitch_accel.add_data_point(phi_ddot);

        wheel_angle.add_data_point(wheel_position);
        wheel_rate.add_data_point(wheel_velocity);
        wheel_accel.add_data_point(theta_ddot);

        torque_command.add_data_point(command);
        bool logSuccess = time_data.add_data_point(time);

        if(!logSuccess)
        {
            // Flush to serial
            imu_pitch.output_data_stream("");
            imu_pitch_rate.output_data_stream("");
            imu_pitch_accel.output_data_stream("");

            wheel_angle.output_data_stream("");
            wheel_rate.output_data_stream("");
            wheel_accel.output_data_stream("");

            torque_command.output_data_stream("");
            time_data.output_data_stream("");

            close("closed_loop");

        }
    }

    last_pitch = pitch;
    last_time = time;
    last_dt = dt;
    last_pitch_rate = dpdt;
    last_wheel_velocity = wheel_velocity;

    i++;

}