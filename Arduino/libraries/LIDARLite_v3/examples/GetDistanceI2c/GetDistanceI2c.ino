#include <LIDARLite.h>

#include <LIDARLite.h>

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
#define LIDAR_1_ADDRESS   DEFAULT_ADDRESS
#define LIDAR_2_ADDRESS    0x64
#define LIDAR_3_ADDRESS    0x66
#define LIDAR_4_ADDRESS    0x68
#define LIDAR_1_ENABLE_PIN 20
#define LIDAR_2_ENABLE_PIN 21
#define LIDAR_3_ENABLE_PIN 22
#define LIDAR_4_ENABLE_PIN 23

LIDARLite Lidar_1, Lidar_2;

void setup()
{
  Serial.begin(115200); // Initialize serial connection to display distance readings
  Serial.print("starting lidar app");
  pinMode(LIDAR_1_ENABLE_PIN, OUTPUT);
  pinMode(LIDAR_2_ENABLE_PIN, OUTPUT);
  pinMode(LIDAR_3_ENABLE_PIN, OUTPUT);
  pinMode(LIDAR_4_ENABLE_PIN, OUTPUT);
  digitalWrite(LIDAR_1_ENABLE_PIN, LOW);
  digitalWrite(LIDAR_2_ENABLE_PIN, LOW);
  digitalWrite(LIDAR_3_ENABLE_PIN, LOW);
  digitalWrite(LIDAR_4_ENABLE_PIN, LOW);
  // Configure Lidar_1
  digitalWrite(LIDAR_1_ENABLE_PIN, HIGH);
  Lidar_1.begin(0, true, DEFAULT_ADDRESS); // Set configuration to default and I2C to 400 kHz
  Lidar_1.changeAddress(LIDAR_1_ADDRESS, true, DEFAULT_ADDRESS);
  Lidar_1.configure(0, LIDAR_1_ADDRESS); // Change this number to try out alternate configurations
  digitalWrite(LIDAR_1_ENABLE_PIN, LOW);
  //Configure Lidar_2
  digitalWrite(LIDAR_2_ENABLE_PIN, HIGH);
  //Lidar_2.begin(0, true, LIDAR_2_ADDRESS); // Set configuration to default and I2C to 400 kHz
  //Lidar_2.changeAddress(LIDAR_2_ADDRESS, true, DEFAULT_ADDRESS);
  //Lidar_2.configure(0, LIDAR_2_ADDRESS); // Change this number to try out alternate configurations
  digitalWrite(LIDAR_2_ENABLE_PIN, LOW);
  //Enable all Lidars
  digitalWrite(LIDAR_1_ENABLE_PIN, HIGH);
  digitalWrite(LIDAR_2_ENABLE_PIN, HIGH);
  digitalWrite(LIDAR_3_ENABLE_PIN, HIGH);
  digitalWrite(LIDAR_4_ENABLE_PIN, HIGH);
  
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
}

void loop()
{
  char output_string[128];
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
  sprintf(output_string, "1: %i\n", Lidar_1.distance(true, LIDAR_1_ADDRESS));
  Serial.print(output_string);

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for(int i = 0; i < 99; i++)
  {
    //Serial.printf("L: %i R: %i", leftLidar.distance(false, LEFT_ADDRESS), rightLidar.distance(false, RIGHT_ADDRESS));
    //Serial.printf("R: %i\n", rightLidar.distance(false, RIGHT_ADDRESS));
    sprintf(output_string, "1: %i\n", Lidar_1.distance(true, LIDAR_1_ADDRESS));
    Serial.print(output_string);
  }
}
