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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.Robot;
import frc.robot.lib.MedianFilter;


public class DriveToVisionTarget extends Command { 

  private static double kP = 0.0; 
  private static double kI = 0.0; 
  private static double kD = 0.0; 
  private static final String keyP = "DriveToVisionTarget_P"; 
  private static final String keyI = "DriveToVisionTarget_I"; 
  private static final String keyD = "DriveToVisionTarget_D"; 

  private MedianFilter median = new MedianFilter(7);

  private PIDController pidController; 
  public DriveToVisionTarget(){
    this(0.0);
  }
  public DriveToVisionTarget(double targetAngle) {
    requires(Robot.driveBase); 
    requires(Robot.vision);  

 
    Preferences prefs = Preferences.getInstance();  
    if(!prefs.containsKey(keyP)) { 
      prefs.putDouble(keyP, 0.0); 
    } 
    if(!prefs.containsKey(keyI)) { 
      prefs.putDouble(keyI, 0.0); 
    }  
    if(!prefs.containsKey(keyD)){ 
      prefs.putDouble(keyD, 0.0);        
    }
    
    kP = prefs.getDouble(keyP, kP); 
    kI = prefs.getDouble(keyI, kI); 
    kD = prefs.getDouble(keyD, kD);  

    Robot.vision.setLineTarget();
    pidController=new PIDController( kP,  kI,  kD,  0.0, 
                                    new PIDSource(){
                                      private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
                                      @Override
                                      public void setPIDSourceType(PIDSourceType pidSource) {
                                        pidSourceType = pidSource;
                                      }
                                    
                                      @Override
                                      public double pidGet() {
                                        return median.filter(Robot.vision.getLineTarget());
                                      }
                                    
                                      @Override
                                      public PIDSourceType getPIDSourceType() {
                                        return pidSourceType;
                                      }
                                    },
                                    new PIDOutput(){
                                    
                                      @Override
                                      public void pidWrite(double output) {
                                        if(Robot.vision.isTargetAcquired()){
                                          Robot.driveBase.driveRobot(-Robot.m_oi.driveY(), -output);
                                        } else{
                                          Robot.driveBase.driveRobot(-Robot.m_oi.driveY(), Robot.m_oi.driveX());
                                        }
                                       
                                      }
                                    });

  pidController.setName("Vision Target PID"); 
  pidController.setInputRange(-27.0, 27.0); 
  pidController.setOutputRange(-1.0, 1.0); 
  pidController.setAbsoluteTolerance(1.0);
  pidController.setSetpoint(targetAngle);
  pidController.setEnabled(false);
  LiveWindow.add(pidController);
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    median.reset();
    pidController.reset();
    pidController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_oi.isDriveToVisionTargetReleased();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pidController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
