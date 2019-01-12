/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveBaseController;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  WPI_TalonSRX talonDriveFrontLeft = new WPI_TalonSRX(RobotMap.talonDriveFrontLeft);
  WPI_TalonSRX talonDriveBackLeft = new WPI_TalonSRX(RobotMap.talonDriveBackLeft);
  WPI_TalonSRX talonDriveFrontRight = new WPI_TalonSRX(RobotMap.talonDriveFrontRight);
  WPI_TalonSRX talonDriveBackRight = new WPI_TalonSRX(RobotMap.talonDriveBackRight);
  WPI_TalonSRX talonDriveBackRightSlave = new WPI_TalonSRX(RobotMap.talonDriveBackRightSlave);
  WPI_TalonSRX talonDriveBackLeftSlave = new WPI_TalonSRX(RobotMap.talonDriveBackLeftSlave);
  WPI_TalonSRX talonDriveFrontLeftSlave = new WPI_TalonSRX(RobotMap.talonDriveFrontLeftSlave);
  WPI_TalonSRX talonDriveFrontRightSlave = new WPI_TalonSRX(RobotMap.talonDriveFrontRightSlave);

  MecanumDrive driveBase = new MecanumDrive(talonDriveFrontLeft, talonDriveBackLeft, talonDriveFrontRight, talonDriveBackRight);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DriveSubsystem(){
    configCurrentLimit(talonDriveFrontLeft);
    configCurrentLimit(talonDriveBackLeft);
    configCurrentLimit(talonDriveFrontRight);
    configCurrentLimit(talonDriveBackRight);
    configCurrentLimit(talonDriveFrontLeftSlave);
    configCurrentLimit(talonDriveBackLeftSlave);
    configCurrentLimit(talonDriveFrontRightSlave);
    configCurrentLimit(talonDriveBackRightSlave);

    talonDriveFrontLeftSlave.set(ControlMode.Follower, RobotMap.talonDriveFrontLeft);
    talonDriveBackLeftSlave.set(ControlMode.Follower, RobotMap.talonDriveBackLeft);
    talonDriveFrontRightSlave.set(ControlMode.Follower, RobotMap.talonDriveFrontRight);
    talonDriveBackRightSlave.set(ControlMode.Follower, RobotMap.talonDriveBackRight);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveBaseController());
  }

  public void configCurrentLimit(WPI_TalonSRX talonSRX){
    talonSRX.configPeakCurrentDuration(50, 10);
    talonSRX.configPeakCurrentLimit(40,10);
    talonSRX.configContinuousCurrentLimit(35, 10);
    talonSRX.enableCurrentLimit(true);
  }

  public void driveRobot(double ySpeed, double xSpeed, double zRotation){
    driveBase.driveCartesian(ySpeed, xSpeed, zRotation);
  }
}
