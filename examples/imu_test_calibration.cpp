#include <Arduino.h>
#include "Options.h"
#include "drive_base.h"
#include "imu.h"

// Create a start-up menu with 3 non-default options
Options options(1);
bool shouldCalibrate;

Imu::Imu imu = Imu::Imu();
void setup()
{
    // start serial
    Serial.begin(115200);

    // Add options to the menu
    options.addOption("Calibrate IMU");
    options.run();

    // Get the selected option
    int selected_option = options.getSelectedOption();
    shouldCalibrate = selected_option == 0;

    imu.init(shouldCalibrate);

}


int loopcount = 0;
void loop()
{

    imu.loop();

    Serial.print("Roll: ");
    Serial.print(imu.getRoll());
    Serial.print(" Pitch: ");
    Serial.print(imu.getPitch());
    Serial.print(" Yaw: ");
    Serial.println(imu.getYaw());

}
