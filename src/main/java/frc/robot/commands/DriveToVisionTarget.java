/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.Robot;

public class DriveToVisionTarget extends Command { 
  private PIDController pidController; 
  public DriveToVisionTarget() {
    requires(Robot.driveBase); 
    requires(Robot.vision);
    Robot.vision.setLineTarget();
    pidController=new PIDController(  0.0,  0.0,  0.0,  0.0, 
                                    new PIDSource(){
                                      private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
                                      @Override
                                      public void setPIDSourceType(PIDSourceType pidSource) {
                                        pidSourceType = pidSource;
                                      }
                                    
                                      @Override
                                      public double pidGet() {
                                        return Robot.vision.getLineTarget();
                                      }
                                    
                                      @Override
                                      public PIDSourceType getPIDSourceType() {
                                        return pidSourceType;
                                      }
                                    },
                                    new PIDOutput(){
                                    
                                      @Override
                                      public void pidWrite(double output) {
                                        Robot.driveBase.driveRobot(-Robot.m_oi.driveY(), -output);
                                      }
                                    });

  pidController.setName("Vision Target PID"); 
  pidController.setInputRange(-27.0, 27.0); 
  pidController.setOutputRange(-1.0, 1.0); 
  pidController.setAbsoluteTolerance(1.0);
  pidController.setSetpoint(0.0);
  pidController.setEnabled(false);
  LiveWindow.add(pidController);
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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