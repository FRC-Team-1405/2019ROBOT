/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClawController extends Command {
  boolean isFrontClawOpen = false;
  boolean isBackClawOpen = false;
  final double intakeSpeed =  1.0;
  final double outputSpeed =  1.0;

  boolean isIntakeActive = false;
    
  public ClawController() {
    requires(Robot.claw);
  }

  public void intake() {
    
  } 
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_oi.topClawOpenPressed()){
      if (isFrontClawOpen) {
        Robot.claw.closeClawTop();
        isFrontClawOpen = false;
      } else {
        Robot.claw.openClawTop();
        isFrontClawOpen = true;
      } 
    }
    if(Robot.m_oi.bottomClawOpenPressed()){
      if(isBackClawOpen){
        Robot.claw.closeClawBottom();
        isBackClawOpen = false;
      } else{
        Robot.claw.openClawBottom();
        isBackClawOpen = true;
      }
    }
    if (Robot.m_oi.cargoOutputPressed()) {
      isIntakeActive = true;
      Robot.claw.intakeCargo(intakeSpeed);
    } else if (Robot.m_oi.cargoIntakePressed()) {
      isIntakeActive = true;
      Robot.claw.releaseCargo(outputSpeed);
    } else if (isIntakeActive){
      isIntakeActive = false;
      Robot.claw.stopCargo();
    }
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
  
  // intake = set claw intakes speed to 1
  
  
}
