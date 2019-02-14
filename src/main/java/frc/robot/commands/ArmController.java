/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmController extends Command {
  public ArmController() {
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.rocketCenter()){
      Robot.arm.rocketCenter();
    } else if(Robot.m_oi.cargoShipTop()){
      Robot.arm.cargoShipTop();
    } else if(Robot.m_oi.backRocketCenter()){
      Robot.arm.rocketCenter();//change this
    } else if(Robot.m_oi.backCargoShipTop()){
      Robot.arm.cargoShipTop();//change this
    }else if(Robot.m_oi.armFloorPressed()) {
      Robot.arm.floor();
    } else if(Robot.m_oi.armLowPressed()){
      Robot.arm.low();
    } else if(Robot.m_oi.backArmFloorPressed()) {
      Robot.arm.floor();//change this
    } else if(Robot.m_oi.backArmLowPressed()){
      Robot.arm.low();//change this
    }
      // Robot.arm.setArmPosition(Robot.m_oi.moveArm());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
