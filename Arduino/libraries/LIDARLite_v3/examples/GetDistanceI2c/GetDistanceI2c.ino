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

#define DEFAULT_ADDRESS 0x62
#define LEFT_ADDRESS    0x64
#define RIGHT_ADDRESS   DEFAULT_ADDRESS

LIDARLite leftLidar, rightLidar;

void setup()
{
  byte regRead[2];
  Serial.begin(115200); // Initialize serial connection to display distance readings
  rightLidar.begin(0, true, RIGHT_ADDRESS); // Set configuration to default and I2C to 400 kHz
  // step 1: turn right LIDAR off
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  // step 2: send left LIDAR commands to registers 0x18 and 0x19 to change I2C address
  leftLidar.changeAddress(LEFT_ADDRESS, true, DEFAULT_ADDRESS);
  // step 3: turn right LIDAR on
  digitalWrite(22, HIGH);
  /*
    begin(int configuration, bool fasti2c, char lidarliteAddress)

    Starts the sensor and I2C.

    Parameters
    ----------------------------------------------------------------------------
    configuration: Default 0. Selects one of several preset configurations.
    fasti2c: Default 100 kHz. I2C base frequency.
      If true I2C frequency is set to 400kHz.
    lidarliteAddress: Default DEFAULT_ADDRESS. Fill in new address here if changed. See
      operating manual for instructions.
  */
  //leftLidar.begin(0, true, LEFT_ADDRESS); // Set configuration to default and I2C to 400 kHz
  
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
    lidarliteAddress: Default DEFAULT_ADDRESS. Fill in new address here if changed. See
      operating manual for instructions.
  */
  leftLidar.configure(0, LEFT_ADDRESS); // Change this number to try out alternate configurations
  //rightLidar.configure(0, RIGHT_ADDRESS); // Change this number to try out alternate configurations
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
    lidarliteAddress: Default DEFAULT_ADDRESS. Fill in new address here if changed. See
      operating manual for instructions.
  */

  // Take a measurement with receiver bias correction and print to serial terminal
  //Serial.printf("R: %i\n", rightLidar.distance(true, RIGHT_ADDRESS));
  Serial.printf("L: %i\n", leftLidar.distance(true, LEFT_ADDRESS));

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for(int i = 0; i < 99; i++)
  {
    //Serial.printf("L: %i R: %i", leftLidar.distance(false, LEFT_ADDRESS), rightLidar.distance(false, RIGHT_ADDRESS));
    //Serial.printf("R: %i\n", rightLidar.distance(false, RIGHT_ADDRESS));
    Serial.printf("L: %i\n", leftLidar.distance(false, LEFT_ADDRESS));
  }
}
