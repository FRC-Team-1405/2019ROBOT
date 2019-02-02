/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.RobotMap;
import frc.robot.commands.DriveBaseController;
//import frc.robot.commands.*;
//import frc.robot.lib.ExtendedTalon;
import frc.robot.lib.TalonPID;
import edu.wpi.first.wpilibj.smartdashboard.*;

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
  TalonPID leftTalonPID;
  TalonPID rightTalonPID;

   public ArcadeDrive(){
      configCurrentLimit(talonDriveBaseLeft);
      configCurrentLimit(talonDriveBaseRight);
      configCurrentLimit(talonDriveBaseLeftSlave);
      configCurrentLimit(talonDriveBaseRightSlave);

      // limit Talon deadband
      talonDriveBaseLeft.configNeutralDeadband(0.001, 10);
      talonDriveBaseRight.configNeutralDeadband(0.001, 10);

      resetDistanceEncoder();

      talonDriveBaseLeftSlave.follow(talonDriveBaseLeft);
      talonDriveBaseRightSlave.follow(talonDriveBaseRight);

      talonDriveBaseLeft.setName("Left");
      talonDriveBaseRight.setName("Right");

      driveBase.setDeadband(0.0);
      this.addChild(driveBase);

      leftTalonPID = new TalonPID(talonDriveBaseLeft, ControlMode.Position);
      leftTalonPID.setName("Left PID");
      LiveWindow.add(leftTalonPID);
      rightTalonPID = new TalonPID(talonDriveBaseRight, ControlMode.Position);
      rightTalonPID.setName("Right PID");
      LiveWindow.add(rightTalonPID);
   }

  private void resetDistanceEncoder() {
    talonDriveBaseLeft.setSelectedSensorPosition(0);
    talonDriveBaseRight.setSelectedSensorPosition(0);
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

  public void setTargetPosition(double pos) {
    leftTalonPID.setSetpoint(pos);
    rightTalonPID.setSetpoint(pos);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Left Pos", () -> { return talonDriveBaseLeft.getSelectedSensorPosition(); }, null );
    builder.addDoubleProperty("Right Pos", () -> { return talonDriveBaseRight.getSelectedSensorPosition(); }, null );
    builder.addDoubleProperty("Target Pos", null, this::setTargetPosition);
  }
}
