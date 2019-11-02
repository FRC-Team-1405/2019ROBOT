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
import frc.robot.RobotMap;

public class DriveToDistance extends Command {

  private static double distance = 10.0;
  private static double kLeftP = 0.0;
  private static double kLeftI = 0.0;
  private static double kLeftD = 0.0;
  private static double kRightP = 0.0;
  private static double kRightI = 0.0;
  private static double kRightD = 0.0;
  private static final String keyLeftP = "Left P";
  private static final String keyLeftI = "Left I";
  private static final String keyLeftD = "Left D";
  private static final String keyRightP = "Right P";
  private static final String keyRightI = "Right I";
  private static final String keyRightD = "Right D";

  private static double leftPIDOutput = 0.0;
  private static double rightPIDOutput = 0.0;

  private PIDController leftPIDController;
  private PIDController rightPIDController;

  public DriveToDistance() {
    requires(Robot.driveBase);

    Preferences prefs = Preferences.getInstance();
    if(!prefs.containsKey(keyLeftP)){
      prefs.putDouble(keyLeftP, 0.0);
    }
    if(!prefs.containsKey(keyLeftI)){
      prefs.putDouble(keyLeftI, 0.0);
    }
    if(!prefs.containsKey(keyLeftD)){
      prefs.putDouble(keyLeftD, 0.0);
    }
    if(!prefs.containsKey(keyRightP)){
      prefs.putDouble(keyRightP, 0.0);
    }
    if(!prefs.containsKey(keyRightI)){
      prefs.putDouble(keyRightI, 0.0);
    }
    if(!prefs.containsKey(keyRightD)){
      prefs.putDouble(keyRightD, 0.0);
    }

    kLeftP = prefs.getDouble(keyLeftP, kLeftP);
    kLeftD = prefs.getDouble(keyLeftI, kLeftI);
    kLeftP = prefs.getDouble(keyLeftD, kLeftD);
    kLeftP = prefs.getDouble(keyRightP, kRightP);
    kLeftP = prefs.getDouble(keyRightI, kRightI);
    kLeftP = prefs.getDouble(keyRightD, kRightD);
    leftPIDOutput = 0.0;
    rightPIDOutput = 0.0;
    leftPIDController = new PIDController( kLeftP,  kLeftI,  kLeftD,  0.0, 
                                          new PIDSource(){
                                            private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
                                            @Override
                                            public void setPIDSourceType(PIDSourceType pidSource) {
                                              pidSourceType = pidSource;
                                            }
                                          
                                            @Override
                                            public double pidGet() {
                                              if(Robot.driveBase.getDirection()){
                                                return Robot.lidarReader.getLidarValue(RobotMap.FRONT_LEFT);
                                              }
                                              return Robot.lidarReader.getLidarValue(RobotMap.BACK_LEFT);
                                            }
                                    
                                            @Override
                                            public PIDSourceType getPIDSourceType() {
                                              return pidSourceType;
                                            }
                                          },
                                          new PIDOutput() {

                                            @Override
                                            public void pidWrite(double output) {
                                              leftPIDOutput = output;

                                            }
                                          });

leftPIDController.setName("Vision Target PID"); 
leftPIDController.setInputRange(-27.0, 27.0); 
leftPIDController.setOutputRange(-1.0, 1.0); 
leftPIDController.setAbsoluteTolerance(1.0);
leftPIDController.setSetpoint(distance);
leftPIDController.setEnabled(false);
LiveWindow.add(leftPIDController);

    rightPIDController = new PIDController( kRightP,  kRightI,  kRightD,  0.0, 
                                          new PIDSource(){
                                            private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
                                            @Override
                                            public void setPIDSourceType(PIDSourceType pidSource) {
                                              pidSourceType = pidSource;
                                            }
                                          
                                            @Override
                                            public double pidGet() {
                                              if(Robot.driveBase.getDirection()){
                                                return Robot.lidarReader.getLidarValue(RobotMap.FRONT_RIGHT);
                                              }
                                              return Robot.lidarReader.getLidarValue(RobotMap.BACK_RIGHT);
                                            }
                                    
                                            @Override
                                            public PIDSourceType getPIDSourceType() {
                                              return pidSourceType;
                                            }
                                          },
                                          new PIDOutput() {

                                            @Override
                                            public void pidWrite(double output) {
                                              rightPIDOutput = output;

                                            }
                                          });

rightPIDController.setName("Vision Target PID"); 
rightPIDController.setInputRange(-27.0, 27.0); 
rightPIDController.setOutputRange(-1.0, 1.0); 
rightPIDController.setAbsoluteTolerance(1.0);
rightPIDController.setSetpoint(distance);    
rightPIDController.setEnabled(false);
LiveWindow.add(rightPIDController);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftPIDController.enable();
    rightPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveBase.tankDriveRobot(leftPIDOutput, rightPIDOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!Robot.m_oi.isDriveToDistanceEnabled()){
      return true;
    }else{
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftPIDController.disable();
    rightPIDController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
