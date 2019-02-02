/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  //Talon IDs
  public final static int talonDriveBaseLeft = 1;
  public final static int talonDriveBaseRight = 2;
  public final static int talonDriveBaseLeftSlave1 = 3;
  public final static int talonDriveBaseRightSlave1 = 4;
  public final static int talonDriveBaseLeftSlave2 = 5;
  public final static int talonDriveBaseRightSlave2 = 6;
  public final static int pivotTalon = 7;
  public final static int pivotTalonSlave = 8; 
  public final static int clawTalonA = 9; 
  public final static int clawTalonB = 10;
  //Joystick IDs
  public final static int pilot = 0; 
  
}
