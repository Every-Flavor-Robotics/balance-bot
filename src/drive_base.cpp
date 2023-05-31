#include "drive_base.h"

SPIClass hspi = SPIClass(HSPI);
bool hspi_initialized = false;

Commander command = Commander(Serial);
BLDCMotor motor_left = BLDCMotor(11);
BLDCMotor motor_right = BLDCMotor(11);

void do_target_left(char* cmd) { command.motor(&motor_left, cmd); }
void do_target_right(char* cmd) { command.motor(&motor_right, cmd); }

DriveBase::DriveBase()
    : encoder_left(MagneticSensorMT6701SSI(k_left_enc_cs)),
      driver_left(BLDCDriver6PWM(k_left_gpio_uh, k_left_gpio_ul, k_left_gpio_vh,
                                 k_left_gpio_vl, k_left_gpio_wh,
                                 k_left_gpio_wl)),
      sensor_calibrated_left(CalibratedSensor(encoder_left)),
      encoder_right(MagneticSensorMT6701SSI(k_right_enc_cs)),
      sensor_calibrated_right(CalibratedSensor(encoder_right)),
      driver_right(BLDCDriver6PWM(k_right_gpio_uh, k_right_gpio_ul,
                                  k_right_gpio_vh, k_right_gpio_vl,
                                  k_right_gpio_wh, k_right_gpio_wl))
{
}

void DriveBase::init() { init(false, false); }

void DriveBase::init_helper(BLDCMotor& motor, BLDCDriver6PWM& driver,
                            CalibratedSensor& sensor_calibrated,
                            MagneticSensorMT6701SSI& encoder, const char* name)
{
  // Init encoder
  encoder.init(&hspi);

  // Link encoder to motor
  motor.linkSensor(&encoder);

  // Init driver and link to motor
  driver.voltage_power_supply = k_voltage_power_supply;
  driver.voltage_limit = k_voltage_limit;
  driver.init();
  motor.linkDriver(&driver);

  // Set motor control parameters
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.velocity_limit = k_velocity_limit;
  motor.voltage_limit = k_voltage_limit;
  motor.current_limit = k_current_limit;

  // FOCStudio options
  if (enable_foc_studio)
  {
    // use monitoring
    motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    motor.useMonitoring(Serial);
  }

  // Initialize motor
  motor.init();

  // Calibrate encoders
  sensor_calibrated.voltage_calibration = k_voltage_calibration;
  if (should_calibrate)
  {
    sensor_calibrated.calibrate(motor, name);
  }
  // Use calibration data if it exists
  else if (!sensor_calibrated.loadCalibrationData(motor, name))
  {
    // If no data was found, calibrate the sensor
    sensor_calibrated.calibrate(motor, name);
  }

  // Link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);

  // Init FOC
  motor.initFOC();

  // Print init message
  Serial.print("Initialized ");
  Serial.println(name);
}

void DriveBase::init(bool should_calibrate, bool enable_foc_studio)
{
  // Guard to prevent multiple initializations, which could cause a crash
  if (!hspi_initialized)
  {
    hspi_initialized = true;
    hspi.begin(k_left_enc_scl, k_left_enc_sda, 27, 3);
  }

  this->should_calibrate = should_calibrate;
  this->enable_foc_studio = enable_foc_studio;
  Serial.print("Enable FOC Studio? ");
  Serial.println(enable_foc_studio ? "Yes" : "No");

  // Initialize motors
  init_helper(motor_left, driver_left, sensor_calibrated_left, encoder_left,
              "left");
  init_helper(motor_right, driver_right, sensor_calibrated_right, encoder_right,
              "right");

  // Set PID parameters for both motors
  motor_right.PID_velocity.P = 0.75;
  motor_right.PID_velocity.I = 0.09;
  motor_right.PID_velocity.D = 0.001;
  motor_right.PID_velocity.output_ramp = 10000.0;

  motor_left.PID_velocity.P = 0.75;
  motor_left.PID_velocity.I = 0.09;
  motor_left.PID_velocity.D = 0.001;
  motor_left.PID_velocity.output_ramp = 10000.0;

  // add command to commander
  if (enable_foc_studio)
  {
    command.add('L', do_target_left, (char*)"target");
    command.add('R', do_target_right, (char*)"target");
  }

  motor_left.disable();
  motor_right.disable();
}

void DriveBase::set_target(float target_left, float target_right)
{
  if (!enable_foc_studio)
  {
    motor_left.move(-target_left);
    motor_right.move(target_right);
  }
}

void DriveBase::loop()
{
  motor_left.loopFOC();
  motor_right.loopFOC();

  // this function can be run at much lower frequency than loopFOC()
  motor_left.move();
  motor_right.move();

  // Monitoring, use only if necessary as it slows loop down significantly
  if (enable_foc_studio)
  {
    // user communication
    command.run();

    motor_left.monitor();
    motor_right.monitor();
  }
}

void DriveBase::enable()
{
  motor_left.enable();
  motor_right.enable();
}

void DriveBase::disable()
{
  motor_left.disable();
  motor_right.disable();
}

// Getters
float DriveBase::get_left_position() { return motor_left.shaftAngle(); }
float DriveBase::get_left_velocity() { return motor_left.shaftVelocity(); }
float DriveBase::get_left_voltage() { return motor_left.voltage.q; }

float DriveBase::get_right_position() { return motor_right.shaftAngle(); }
float DriveBase::get_right_velocity() { return motor_right.shaftVelocity(); }
float DriveBase::get_right_voltage() { return motor_right.voltage.q; }