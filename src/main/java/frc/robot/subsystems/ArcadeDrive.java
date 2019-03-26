/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.RobotMap;
import frc.robot.commands.DriveBaseController;
import frc.robot.lib.ExtendedTalon;
//import frc.robot.commands.*;
//import frc.robot.lib.ExtendedTalon;
// import frc.robot.lib.TalonPID;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
/**
 * Add your docs here.
 */
public class ArcadeDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static double speedLimit = 1.0;
  private static final String keySpeedLimit = "ArcadeDrive_SpeedLimit";

  WPI_TalonSRX talonDriveBaseLeft = new WPI_TalonSRX(RobotMap.talonDriveBaseLeft);
  WPI_TalonSRX talonDriveBaseRight = new WPI_TalonSRX(RobotMap.talonDriveBaseRight);
  TalonSRX talonDriveBaseLeftSlave1 = new TalonSRX(RobotMap.talonDriveBaseLeftSlave1);
  TalonSRX talonDriveBaseRightSlave1 = new TalonSRX(RobotMap.talonDriveBaseRightSlave1);
  TalonSRX talonDriveBaseLeftSlave2 = new TalonSRX(RobotMap.talonDriveBaseLeftSlave2);
  TalonSRX talonDriveBaseRightSlave2 = new TalonSRX(RobotMap.talonDriveBaseRightSlave2);
  DifferentialDrive driveBase = new DifferentialDrive(talonDriveBaseLeft, talonDriveBaseRight); 
  // TalonPID leftTalonPID;
  // TalonPID rightTalonPID;
  boolean driveForward = true;

   public ArcadeDrive(){
     ExtendedTalon.configCurrentLimit(talonDriveBaseLeft);
     ExtendedTalon.configCurrentLimit(talonDriveBaseRight);
     ExtendedTalon.configCurrentLimit(talonDriveBaseLeftSlave1);
     ExtendedTalon.configCurrentLimit(talonDriveBaseRightSlave1);
     ExtendedTalon.configCurrentLimit(talonDriveBaseLeftSlave2);
     ExtendedTalon.configCurrentLimit(talonDriveBaseRightSlave2); 

      // limit Talon deadband
      talonDriveBaseLeft.configNeutralDeadband(0.04, 10);
      talonDriveBaseRight.configNeutralDeadband(0.04, 10);

      resetDistanceEncoder();
      
      talonDriveBaseLeft.set(ControlMode.PercentOutput, 0);
      talonDriveBaseRight.set(ControlMode.PercentOutput, 0);
      talonDriveBaseLeftSlave1.set(ControlMode.Follower, RobotMap.talonDriveBaseLeft);
      talonDriveBaseRightSlave1.set(ControlMode.Follower, RobotMap.talonDriveBaseRight);
      talonDriveBaseLeftSlave2.set(ControlMode.Follower, RobotMap.talonDriveBaseLeft);
      talonDriveBaseRightSlave2.set(ControlMode.Follower, RobotMap.talonDriveBaseRight);

      talonDriveBaseLeft.setName("Left");
      talonDriveBaseRight.setName("Right");

      driveBase.setDeadband(0.0);
      this.addChild(driveBase);

      Preferences prefs = Preferences.getInstance();

      if(!prefs.containsKey(keySpeedLimit)) {
        prefs.putDouble(keySpeedLimit, speedLimit);
      }

      speedLimit = prefs.getDouble(keySpeedLimit, speedLimit);

      // leftTalonPID = new TalonPID(talonDriveBaseLeft, ControlMode.Position);
      // leftTalonPID.setName("Left PID");
      // LiveWindow.add(leftTalonPID);
      // rightTalonPID = new TalonPID(talonDriveBaseRight, ControlMode.Position);
      // rightTalonPID.setName("Right PID");
      // LiveWindow.add(rightTalonPID);
   }

  private void resetDistanceEncoder() {
    talonDriveBaseLeft.setSelectedSensorPosition(0);
    talonDriveBaseRight.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveBaseController());
  }

  public void toggleDriveDirection(){
    driveForward = !driveForward;
  }

  public void driveRobot(double xSpeed, double zRotation){
    if(driveForward){
      driveBase.arcadeDrive(xSpeed*speedLimit, zRotation*speedLimit);
    } else{
      driveBase.arcadeDrive(-xSpeed*speedLimit, zRotation*speedLimit);
    }
  }

  public void tankDriveRobot(double leftSpeed, double rightSpeed){
    if(driveForward){
      driveBase.tankDrive(leftSpeed*speedLimit, rightSpeed*speedLimit);
    } else{
      driveBase.tankDrive(-leftSpeed*speedLimit, -rightSpeed*speedLimit);
    }
  }

  public boolean getDirection(){
    return driveForward;
  }

  // public void setTargetPosition(double pos) {
  //   leftTalonPID.setSetpoint(pos);
  //   rightTalonPID.setSetpoint(pos);
  // }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Drive Forward", () -> {return this.driveForward;}, null);
  }
}
