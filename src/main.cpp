#include <Arduino.h>
#include "Options.h"
#include "drive_base.h"

// Create a start-up menu with 3 non-default options
Options options(3);
bool shouldCalibrate;
bool enableFocStudio;

// Create a drive base object
DriveBase drive_base = DriveBase();

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
    shouldCalibrate = selected_option == 0 || selected_option == 2;
    enableFocStudio = selected_option == 1 || selected_option == 2;

    drive_base.init(shouldCalibrate, enableFocStudio);

    _delay(1000);
}

void loop()
{
    drive_base.loop();
}