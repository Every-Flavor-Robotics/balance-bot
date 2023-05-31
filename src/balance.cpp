#include "Arduino.h"
#include "Options.h"
#include "common/pid.h"
#include "drive_base.h"
#include "imu.h"

// Create a start-up menu with 3 non-default options
Options options(4);
bool should_calibrate_motors;
bool enable_foc_studio;
bool should_calibrate_imu;

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
  options.addOption("Recalibrate motor");
  options.addOption("FOCStudio");
  options.addOption("Recalibrate motor + FOCStudio");
  options.addOption("Calibrate IMU");
  options.run();

  drive_base = new DriveBase();

  int selected_option = options.getSelectedOption();

  should_calibrate_motors = selected_option == 0 || selected_option == 2;
  enable_foc_studio = selected_option == 1 || selected_option == 2;
  should_calibrate_imu = selected_option == 3;

  drive_base->init(should_calibrate_motors, enable_foc_studio);
  imu.init(false);

  delay(1000);
  drive_base->enable();

  Serial.println("Letting IMU stabilize...");
  int stable_count = 0;
  while (stable_count < 50)
  {
    imu.loop();
    if (imu.get_pitch() > 0.02f || imu.get_pitch() < -0.02f)
    {
      stable_count = 0;
    }
    else
    {
      stable_count++;
    }
  }

  float last_pitch = imu.get_pitch();
  Serial.println("Starting loop...");
  zero_wheel_left = drive_base->get_left_position() - last_pitch;
  zero_wheel_right = drive_base->get_right_position() + last_pitch;

  zero_yaw = imu.get_yaw();

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

  // float dt = time - last_time;
  float pitch = (imu.get_pitch());
  // Convert deg/s to rad/s
  float pitch_rate = imu.get_raw_gyro_y() * 0.0174533f;

  float wheel_position =
      ((drive_base->get_left_position() - zero_wheel_left) -
       (drive_base->get_right_position() - zero_wheel_right)) /
      2;

  float wheel_velocity =
      (drive_base->get_left_velocity() - drive_base->get_right_velocity()) / 2;

  // u = -kx
  float command;
  float command_steer;
  if (desired_vel > 0.1f || desired_vel < -0.1f)
  {
    command = -(k_0 * (wheel_position + pitch - desired_pos) +
                k_1 * (wheel_velocity + pitch_rate - desired_vel) +
                k_2 * pitch + k_3 * pitch_rate);
    command_steer = steering_pid(imu.get_yaw() - zero_yaw);
  }
  else
  {
    desired_pos = 0;
    command = -(standing_k_0 * (wheel_position + pitch) +
                standing_k_1 * (wheel_velocity + pitch_rate) +
                standing_k_2 * pitch + standing_k_3 * pitch_rate);
    command_steer = steering_pid(imu.get_yaw() - zero_yaw);
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

  drive_base->set_target(command - command_steer, command + command_steer);
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