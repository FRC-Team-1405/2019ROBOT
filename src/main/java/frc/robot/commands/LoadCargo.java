/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.ArmController.ArmPosition;

public class LoadCargo extends Command {
  public LoadCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.claw);
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.claw.intakeCargo(1.0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // How do we know the cargo has been loaded?
    if (Robot.m_oi.isLoadCargoReleased()) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.claw.intakeCargo(0.0);
    // if front cargo ship position is closer than farther cargo ship position,
    if (Math.abs(Robot.arm.getArmPosition()-Robot.arm.getCargoShipCargoPos(false)) <
        Math.abs(Robot.arm.getArmPosition()-Robot.arm.getCargoShipCargoPos(true))) {
      // go to front position (b/c it's closer to go to front)
      Robot.arm.frontCargoShipTop();
    } else {
      // go to back position (b/c it's closer to go to back)
      Robot.arm.backCargoShipTop();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
