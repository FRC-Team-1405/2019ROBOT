/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class GyroSystem extends Subsystem {
  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  
  public GyroSystem(){
    addChild(gyro);
    SmartDashboard.putData("NavX", gyro);
  }

  @Override
  public void initDefaultCommand() {

  }

  public double getGyroAngle(){
    return Math.IEEEremainder(gyro.getAngle(), 360.0);
  }

  public void resetGyro(){
    gyro.reset();
  }

  private boolean isReady(){
    return !gyro.isCalibrating();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    if(Robot.limitDebug)
      return;

  // Shuffleboard Debug

    builder.addBooleanProperty("Ready", this::isReady, null);

    builder.addDoubleProperty("Pitch", gyro::getPitch, null);
    builder.addDoubleProperty("Roll",  gyro::getRoll,  null);
    builder.addDoubleProperty("Yaw",   gyro::getYaw,   null);
  }

}
