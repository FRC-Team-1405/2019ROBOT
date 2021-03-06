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
    Robot.claw.stopHatch();
    if(Robot.arm.armInPosition() == ArmPosition.FLOOR_FRONT){
      Robot.arm.frontCargoShipTop();
      Robot.claw.intakeHatch(1.0);
      //Robot.claw.closeClawTop();
    } else{
        Robot.arm.backCargoShipTop();
        Robot.claw.intakeHatch(1.0);
        System.out.println("Arm up");
        //Robot.claw.closeClawBottom();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.m_oi.isLoadHatchReleased() 
          || Robot.arm.armInPosition() == ArmPosition.CARGO_SHIP_FRONT 
          || Robot.arm.armInPosition() == ArmPosition.CARGO_SHIP_BACK){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.claw.stopHatch();
    Robot.claw.closeClawTop(); 
    Robot.claw.closeClawBottom(); 
    Robot.claw.holdHatch();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.claw.stopHatch();
  }
}
