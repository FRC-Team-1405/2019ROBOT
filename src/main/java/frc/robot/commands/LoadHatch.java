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

public class LoadHatch extends Command {
  public LoadHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
    requires(Robot.claw);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.arm.frontCargoShipTop();
    Robot.claw.intakeCargo(1.0);
    Robot.claw.closeClawTop();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //System.err.printf("LoadHatch %b %b\n", Robot.m_oi.isLoadHatchReleased(), (Robot.arm.armInPosition() == ArmPosition.CARGO_SHIP_FRONT) );
    return (Robot.m_oi.isLoadHatchReleased() || Robot.arm.armInPosition() == ArmPosition.CARGO_SHIP_FRONT);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.claw.intakeCargo(0.0);
    Robot.claw.closeClawBottom(); 
    Robot.claw.intakeCargo(.1); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.claw.intakeCargo(0.0);
  }
}
