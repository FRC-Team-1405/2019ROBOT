/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public XboxController pilot = new XboxController(RobotMap.pilot);
  private Limelight limelight = new Limelight();
  public boolean cargoIntakePressed() {
    return pilot.getBumperPressed(Hand.kLeft);
  }
  public boolean cargoOutputPressed() {
    return pilot.getBumperPressed(Hand.kRight);
  }

  public boolean clawOpenPressed(){
    return pilot.getAButtonPressed();
  }

  public double driveY(){
    return pilot.getY(Hand.kLeft);
  }

  public double driveX(){
    return pilot.getX(Hand.kRight);
  }

  public boolean pivotArm(){
    return pilot.getBButton();
  }

  public double visionTargetAngle(){ 
    return limelight.getTX(); 

  }
}
