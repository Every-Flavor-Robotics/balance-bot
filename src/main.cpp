#include <SimpleFOC.h>

// Encoder libs
#include <sensors/MagneticSensorI2C.h>
#include "mt6701_sensor.h"
#include "encoders/calibrated/CalibratedSensor.h"
#include <Wire.h>
#include "Options.h"

// Create a start-up menu with 3 non-default options
Options options(3);
bool shouldCalibrate;
bool enableFocStudio;

// Encoder pins
const int k_enc_sda = 21;
const int k_enc_scl = 22;
const int k_enc_cs  = 32;

// Motor pins
const int k_gpio_uh = 33;
const int k_gpio_ul = 25;
const int k_gpio_vh = 26;
const int k_gpio_vl = 27;
const int k_gpio_wh = 14;
const int k_gpio_wl = 12;

// target velocities will be calculated as a percentage of the max velocity
constexpr float max_velocity_rad_per_sec = 25.0f;

// Encoder, motor, and driver instances
MT6701Sensor encoder = MT6701Sensor();
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(k_gpio_uh, k_gpio_ul, k_gpio_vh, k_gpio_vl, k_gpio_wh, k_gpio_wl);
// Create a calibrated sensor instance and pass it the encoder
CalibratedSensor sensor_calibrated = CalibratedSensor(encoder);

// commander instance
Commander command = Commander(Serial);
void doTarget(char* cmd){command.motor(&motor, cmd);}

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
    shouldCalibrate = selected_option == 0 || selected_option == 2;
    enableFocStudio = selected_option == 1 || selected_option == 2;

    // Initialize encoder
    encoder.init(k_enc_scl, k_enc_sda, k_enc_cs);
    // link motor to sensor
    motor.linkSensor(&encoder);

    // Initialize driver
    driver.voltage_power_supply = 5.0;
    driver.voltage_limit = 5.0;
    driver.init();
    // Link driver to motor
    motor.linkDriver(&driver);

    // set motion control type to velocity
    motor.controller = MotionControlType::velocity;
    // Set Space Vector PWM modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    // set torque control type to voltage (default)
    motor.torque_controller = TorqueControlType::voltage;

    // set motor velocity, voltage, and current limits.

    motor.velocity_limit = 60.0;
    motor.voltage_limit = 7.0;
    motor.current_limit = 1.2;


    // FOCStudio options
    if(enableFocStudio)
    {
        // use monitoring
        motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
        motor.useMonitoring(Serial);
    }

    // initialize motor
    motor.init();

    sensor_calibrated.voltage_calibration = 1;
    // If options 0 or 2 are selected, recalibrate the motor
    if(shouldCalibrate)
    {
        sensor_calibrated.calibrate(motor, "left");
    }
    // Use calibration data if it exists
    else if (!sensor_calibrated.loadCalibrationData(motor, "left"))
    {
        // If no data was found, calibrate the sensor
        sensor_calibrated.calibrate(motor, "left");
    }

    // Link the calibrated sensor to the motor
    motor.linkSensor(&sensor_calibrated);

    // Init FOC
    motor.initFOC();

    // add command to commander
    if(enableFocStudio)
    {
        command.add('M', doTarget, "target");
    }

    _delay(1000);
}

void loop()
{
    motor.loopFOC();

    // this function can be run at much lower frequency than loopFOC()
    motor.move();

    // user communication
    command.run();

    // Monitoring, use only if necessary as it slows loop down significantly
    if(enableFocStudio)
    {
        motor.monitor();
    }

}
