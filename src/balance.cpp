#include "Arduino.h"
#include "Options.h"
#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "drive_base.h"
#include "imu.h"

// Create a start-up menu with 3 non-default options
Options options(4);
bool shouldCalibrateMotors;
bool enableFocStudio;
bool shouldCalibrateImu;

// Create a drive base object
DriveBase* drive_base = nullptr;

// Create IMU
Imu::Imu imu = Imu::Imu();
// low pass
LowPassFilter velocity_command = LowPassFilter(0.3f);

// float k_0 = 0.03535534f;
// float k_1 = 3.11529036f;
// float k_2 = 23.84869906f;
// float k_3 = 14.1f;

// float k_0 = 0.03535534f;
// float k_1 = 3.15890315f;
// float k_2 = 24.29259086f;
// float k_3 = 14.2874019f;

float standing_k_0 = 0.02536067977521028;
float standing_k_1 = 3.258069289823837;
float standing_k_2 = 25.39465377043689;
float standing_k_3 = 15.19592074007437;
float k_0 = 0.040824829046041344;
float k_1 = 3.581318765549753;
float k_2 = 28.127196312812885;
float k_3 = 17.311472756698436;

float zero_wheel_left = 0.0f;
float zero_wheel_right = 0.0f;

float zero_yaw = 0.0f;

float angle_offset = -0.029824;

float target_freq = 250.0f;

float desired_pos = 0;
float desired_vel = 0;

// Steering pid controller
PIDController steering_pid = PIDController(3.2f, 0.0000f, 0.1f, 10000, 10000);

// We need to log the following:
// - time
// - pitch
// - pitch rate
// - pitch accel
// - wheel angle
// - wheel rate
// - wheel accel
// - torque command

int i = 0;

void setup()
{
  // start serial
  Serial.begin(115200);

  // Add options and run menu before setting up motor
  // options.addOption("Recalibrate motor");
  // options.addOption("FOCStudio");
  // options.addOption("Recalibrate motor + FOCStudio");
  // options.addOption("Calibrate IMU");
  // options.run();

  drive_base = new DriveBase();

  int selected_option = options.getSelectedOption();
  // selected_option = 0;

  shouldCalibrateMotors = selected_option == 0 || selected_option == 2;
  enableFocStudio = selected_option == 1 || selected_option == 2;
  shouldCalibrateImu = selected_option == 3;

  drive_base->init(shouldCalibrateMotors, enableFocStudio);
  imu.init(false);

  delay(1000);
  drive_base->enable();

  Serial.println("Letting IMU stabilize...");
  int stable_count = 0;
  while (stable_count < 50)
  {
    imu.loop();
    // pitch_rate_filter(imu.getRawGyroY() * 0.0174533f);
    if (imu.getPitch() > 0.02f || imu.getPitch() < -0.02f)
    {
      stable_count = 0;
    }
    else
    {
      stable_count++;
    }
  }

  float last_pitch = imu.getPitch();
  Serial.println("Starting loop...");
  zero_wheel_left = drive_base->getLeftPosition() - last_pitch;
  zero_wheel_right = drive_base->getRightPosition() + last_pitch;

  zero_yaw = imu.getYaw();

  delay(50);
}

void loop()
{
  // Start time
  unsigned long start_time = micros();

  imu.loop();

  //////// Apply controller

  // Generate state
  // theta (wheel angle)
  // theta_dot
  // phi (pitch)
  // phi_dot

  float time = (float)millis() / 1000.0f;
  // float dt = time - last_time;
  float pitch = (imu.getPitch());
  // Convert deg/s to rad/s
  float pitchRate =
      imu.getRawGyroY() *
      0.0174533f;  // pitch_rate_filter(imu.getRawGyroY() * 0.0174533f);

  float wheel_position = ((drive_base->getLeftPosition() - zero_wheel_left) -
                          (drive_base->getRightPosition() - zero_wheel_right)) /
                         2;
  float wheel_velocity =
      (drive_base->getLeftVelocity() - drive_base->getRightVelocity()) / 2;

  // u = -kx
  float command;
  float command_steer;
  if (desired_vel > 0.1f || desired_vel < -0.1f)
  {
    command = -(k_0 * (wheel_position + pitch - desired_pos) +
                k_1 * (wheel_velocity + pitchRate - desired_vel) + k_2 * pitch +
                k_3 * pitchRate);
    command_steer = steering_pid(imu.getYaw() - zero_yaw);
  }
  else
  {
    desired_pos = 0;
    command = -(standing_k_0 * (wheel_position + pitch) +
                standing_k_1 * (wheel_velocity + pitchRate) +
                standing_k_2 * pitch + standing_k_3 * pitchRate);
    command_steer = steering_pid(imu.getYaw() - zero_yaw);
  }

  if (i > 8000)
  {
    i = 0;
  }
  else if (i > 7000)
  {
    desired_vel = velocity_command(-15.0f);
  }
  else if (i > 5000)
  {
    desired_vel = velocity_command(0.0f);
    // desired_vel = 0.0f;
  }

  else if (i > 4000)
  {
    desired_vel = velocity_command(15.0f);
  }
  else
  {
    desired_vel = velocity_command(0.0f);
    // desired_vel = 0.0f;
  }

  desired_pos += desired_vel * 0.004f;

  drive_base->setTarget(command - command_steer, command + command_steer);
  drive_base->loop();

  // Sleep to maintain target frequency
  unsigned long end_time = micros();
  unsigned long elapsed_time = end_time - start_time;
  unsigned long target_time = (unsigned long)(1000000.0f / target_freq);
  if (elapsed_time < target_time)
  {
    delayMicroseconds(target_time - elapsed_time);
  }

  i++;
}