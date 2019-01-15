/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  GetDistanceI2c

  This example shows how to initialize, configure, and read distance from a
  LIDAR-Lite connected over the I2C interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND
  
  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

#include <Wire.h>
#include <LIDARLite.h>

LIDARLite leftLidar, rightLidar;

void setup()
{
  Serial.begin(115200); // Initialize serial connection to display distance readings
  // step 1: turn right LIDAR off
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  // step 2: send left LIDAR commands to registers 0x18 and 0x19 to change I2C address
  // Read from 0x16 and write that to 0x18, Read from 0x17 and write that to 0x19 to unlock
  // Write 0x63 (new address) to 0x1a
  leftLidar.write(0x18, 0x00, 0x62);
  leftLidar.write(0x19, 0x63, 0x62);
  // step 3: send left LIDAR command to register 0x1e to switch to non-standard address
  leftLidar.write(0x1e, 0x08, 0x62);
  // step 4: turn right LIDAR on
  digitalWrite(22, HIGH);
  /*
    begin(int configuration, bool fasti2c, char lidarliteAddress)

    Starts the sensor and I2C.

    Parameters
    ----------------------------------------------------------------------------
    configuration: Default 0. Selects one of several preset configurations.
    fasti2c: Default 100 kHz. I2C base frequency.
      If true I2C frequency is set to 400kHz.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  leftLidar.begin(0, true); // Set configuration to default and I2C to 400 kHz
  rightLidar.begin(0, true); // Set configuration to default and I2C to 400 kHz
  
  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  leftLidar.configure(0); // Change this number to try out alternate configurations
  rightLidar.configure(0); // Change this number to try out alternate configurations
}

void loop()
{
  /*
    distance(bool biasCorrection, char lidarliteAddress)

    Take a distance measurement and read the result.

    Parameters
    ----------------------------------------------------------------------------
    biasCorrection: Default true. Take aquisition with receiver bias
      correction. If set to false measurements will be faster. Receiver bias
      correction must be performed periodically. (e.g. 1 out of every 100
      readings).
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */

  // Take a measurement with receiver bias correction and print to serial terminal
  Serial.printf("L: %i R: %i", leftLidar.distance(), rightLidar.distance());

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for(int i = 0; i < 99; i++)
  {
    Serial.printf("L: %i R: %i", leftLidar.distance(false), rightLidar.distance(false));
  }
}
