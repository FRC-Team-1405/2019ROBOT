/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
/**
 * Add your docs here.
 */
public class ArcadeDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX talonDriveBaseLeft = new WPI_TalonSRX(RobotMap.talonDriveBaseLeft);
  WPI_TalonSRX talonDriveBaseRight = new WPI_TalonSRX(RobotMap.talonDriveBaseRight);
  WPI_TalonSRX talonDriveBaseLeftSlave = new WPI_TalonSRX(RobotMap.talonDriveBaseLeftSlave);
  WPI_TalonSRX talonDriveBaseRightSlave = new WPI_TalonSRX(RobotMap.talonDriveBaseRightSlave);
  DifferentialDrive driveBase = new DifferentialDrive(talonDriveBaseLeft, talonDriveBaseRight); 

   public ArcadeDrive(){
      configCurrentLimit(talonDriveBaseLeft);
      configCurrentLimit(talonDriveBaseRight);
      configCurrentLimit(talonDriveBaseLeftSlave);
      configCurrentLimit(talonDriveBaseRightSlave);

      talonDriveBaseLeftSlave.set(ControlMode.Follower, RobotMap.talonDriveBaseLeft);
      talonDriveBaseRightSlave.set(ControlMode.Follower, RobotMap.talonDriveBaseRight);

   }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveBaseController());
  }

  public void configCurrentLimit(WPI_TalonSRX talonSRX){
    talonSRX.configPeakCurrentDuration(50, 10);
    talonSRX.configPeakCurrentLimit(40, 10);
    talonSRX.configContinuousCurrentLimit(35, 10);
    talonSRX.enableCurrentLimit(true);

  }

  public void driveRobot(double xSpeed, double zRotation){
    driveBase.arcadeDrive(xSpeed, zRotation);
  }
}
