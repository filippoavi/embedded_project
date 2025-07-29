This is the repository for the final project of the Embedded Systems course.

## How to Install PlatformIO

Go to the Extensions view by clicking the square icon in the sidebar or pressing `Ctrl+Shift+X`.
Search for "PlatformIO IDE" and install it.
After installation, reload VS Code if prompted.

For more details, visit [PlatformIO official website](https://platformio.org).

## How to Install Project Libraries

After installing PlatformIO, you can install the required libraries using the PlatformIO Library Manager. Open the PlatformIO Home, go to "Libraries", and search for each library name. Alternatively, add them to your `platformio.ini` file under the `[env]` section:

```
lib_deps = 
 arduino-libraries/ArduinoBLE@^1.3.7
	greiman/SdFat@^2.3.0
	adafruit/RTClib@^2.1.4
```

Make sure the `framework-arduino-mbed` (Arduino APIs and IDE integration files targeting a generic mbed-enabled board) used is version 4.1.5, as this is the one tested in the project and different versions may not work as intended. If needed add the following lines to the `platformio.ini` file:

```
platform_packages =
 framework-arduino-mbed@4.1.5
```

PlatformIO will automatically download and install these libraries when you build the project.

## Relevant libraries used

  * https://github.com/adafruit/RTClib
  * https://github.com/greiman/SdFat
  * https://github.com/arduino-libraries/ArduinoBLE
  * modified in order to work for the project: https://github.com/arduino-libraries/Arduino_BHY2
  * used only as reference: https://github.com/boschsensortec/BHI2xy_SensorAPI
