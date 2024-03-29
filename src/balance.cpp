#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>

#include "Arduino.h"
#include "Options.h"
#include "common/pid.h"
#include "drive_base.h"
#include "imu.h"

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

TaskHandle_t loop_foc_task;

// Create a start-up menu with 3 non-default options
Options options(4);
bool should_calibrate_motors;
bool enable_foc_studio;
bool should_calibrate_imu;

// Create a drive base object
DriveBase *drive_base = nullptr;

// Create IMU
Imu::Imu imu = Imu::Imu();
// low pass
LowPassFilter velocity_command = LowPassFilter(0.1f);

LowPassFilter is_static_detector = LowPassFilter(1.0f);

// float k_0 = 0.03535534f;
// float k_1 = 3.11529036f;
// float k_2 = 23.84869906f;
// float k_3 = 14.1f;

float k_0 = 0.03535534f;
float k_1 = 3.15890315f;
float k_2 = 24.29259086f;
float k_3 = 14.2874019f;

float standing_k_0 = 0.02536067977521028;
float standing_k_1 = 3.308069289823837;
float standing_k_2 = 25.39465377043689;
float standing_k_3 = 15.20592074007437;
// float k_0 = 0.040824829046041344;
// float k_1 = 3.581318765549753;
// float k_2 = 28.127196312812885;
// float k_3 = 17.311472756698436;

bool pos_reset = false;

float zero_wheel_left = 0.0f;
float zero_wheel_right = 0.0f;

float zero_yaw = 0.0f;
float desired_yaw = 0.0f;

float angle_offset = -0.003;

float desired_pos = 0;
float desired_vel = 0;
float desired_steer_rate = 0;
bool is_static = true;

// Radians per second
const float MAX_VEL = 4.0f;
const float MAX_STEER_RATE = 1.3f;

// Steering pid controller
PIDController steering_pid = PIDController(3.7f, 0.0000f, 0.05f, 10000, 10000);

// Joystick data struct
// uint8_t broadcast_address[] = {0x24, 0x0A, 0xC4, 0x00, 0x4C, 0x6E};
// Create struct for data transmission
const int JOYSTICK_DATA_LEN = (6) * sizeof(int);
typedef union
{
  struct __attribute__((packed))
  {
    int joy1_x;
    int joy1_y;
    int joy1_sw;

    int joy2_x;
    int joy2_y;
    int joy2_sw;
  };

  uint8_t raw[JOYSTICK_DATA_LEN];
} joystick_data_t;

// Callback for receiving joystick data over ESPNow
void on_joystick_data_receive(const uint8_t *mac, const uint8_t *incoming_data,
                              int len);

void loop_foc(void *pvParameters);

unsigned long last_time = 0;
float loop_time = 0;
int samples = 0;
unsigned long last_print_time = 0;
void setup()
{
  // start serial
  Serial.begin(115200);

  delay(3000);

  drive_base = new DriveBase();

  //   int selected_option = options.getSelectedOption();

  bool calibrate_motors = false;
  bool enable_foc_studio = false;
  drive_base->init(calibrate_motors, enable_foc_studio);
  imu.init(false);

  delay(1000);

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

  zero_yaw = 0.0f;  // imu.get_yaw();

  // Setup ESP Now
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(on_joystick_data_receive);

  delay(50);
  loop_time = micros();
  last_print_time = millis();

  xTaskCreatePinnedToCore(
      loop_foc,       /* Task function. */
      "Loop FOC",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &loop_foc_task, /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */

  drive_base->enable();

  last_time = micros();
}

void loop_foc(void *pvParameters)
{
  Serial.print("Loop FOC running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    drive_base->loop();

    //   Print loop frequency
    esp_task_wdt_reset();
  }
}

void loop()
{
  // Start time

  imu.loop();

  //////// Apply controller

  // Generate state
  // theta (wheel angle)
  // theta_dot
  // phi (pitch)
  // phi_dot

  float pitch = (imu.get_pitch() - angle_offset);
  // Convert deg/s to rad/s
  float pitch_rate = imu.get_raw_gyro_y() * 0.0174533f;

  float wheel_position =
      (-(drive_base->get_left_position() - zero_wheel_left) -
       (drive_base->get_right_position() - zero_wheel_right)) /
      2;

  float wheel_velocity =
      (-drive_base->get_left_velocity() - drive_base->get_right_velocity()) / 2;
  // Get yaw from wheel positions
  float yaw = (-(drive_base->get_left_position() - zero_wheel_left) +
               (drive_base->get_right_position() - zero_wheel_right)) /
              2;

  // Get loop time in seconds
  loop_time = (micros() - last_time) / 1000000.0f;
  last_time = micros();
  desired_yaw += desired_steer_rate * loop_time;
  desired_pos += desired_vel * loop_time;

  float filtered_velocity = is_static_detector(abs(wheel_velocity));

  // u = -kx
  float command;
  float command_steer;

  // Compute steering command
  float steer_error = yaw - zero_yaw - desired_yaw;
  command_steer = -steering_pid(steer_error);

  command = -0.85 * (k_0 * (wheel_position + pitch - desired_pos) +
                     k_1 * (wheel_velocity + pitch_rate - desired_vel) +
                     k_2 * pitch + k_3 * pitch_rate);
  if (desired_vel < 0.05f && !pos_reset && desired_steer_rate == 0.0f)
  {
    desired_pos = wheel_position + pitch;
    desired_yaw = yaw - zero_yaw;
    pos_reset = true;
  }
  else if (desired_vel > 0.05f || desired_steer_rate > 0.05f)
  {
    pos_reset = false;
  }

  drive_base->set_target(-command + command_steer, command + command_steer);

  // If the robot has fallen, disable the motors
  if (abs(pitch) > 0.8f)
  {
    drive_base->disable();
  }
  else
  {
    drive_base->enable();
  }

  vTaskDelay(1 / portTICK_PERIOD_MS);
}

// Function definitions
void on_joystick_data_receive(const uint8_t *mac, const uint8_t *incoming_data,
                              int len)
{
  joystick_data_t data;
  memcpy(&data.raw, incoming_data, JOYSTICK_DATA_LEN);

  //   Map joy1_x to desired_vel
  //   Map joy2_y to desired_steer_rate

  //   Deadband
  if (abs(data.joy1_x) < 10)
  {
    data.joy1_x = 0;
  }
  else
  {
    is_static = false;
  }
  if (abs(data.joy2_x) < 10)
  {
    data.joy2_x = 0;
  }
  else
  {
    is_static = false;
  }

  desired_vel = velocity_command(MAX_VEL * ((float)data.joy1_y / 100.0f));
  desired_steer_rate = -MAX_STEER_RATE * ((float)data.joy2_x / 100.0f);
}