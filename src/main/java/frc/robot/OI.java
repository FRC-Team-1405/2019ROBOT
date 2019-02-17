/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public XboxController pilot = new XboxController(RobotMap.pilot);
  public XboxController operator = new XboxController(RobotMap.operator);
  public boolean cargoIntakePressed() {
    return pilot.getBumper(Hand.kLeft);
  }
  public boolean cargoOutputPressed() {
    return pilot.getBumper(Hand.kRight);
  }

  public boolean  frontClawOpenPressed(){
    return pilot.getAButtonPressed();
  }

  public boolean backClawOpenPressed(){
    return pilot.getXButtonPressed();
  }

  public double driveY(){
    return pilot.getY(Hand.kLeft);
  }

  public double driveX(){
    return pilot.getX(Hand.kRight);
  }

  public boolean driveReverse(){
    return pilot.getBButtonPressed();
  }

  public boolean isDriveToLineEnabled(){
    return operator.getBumper(Hand.kRight);
  }

  public boolean armFloorPressed(){
    return (operator.getPOV() == 90);
  }

  public boolean armLowPressed(){
    return (operator.getBumper(Hand.kLeft) && operator.getPOV() == 90);
  }

  public boolean rocketCenter(){
    return (operator.getBumper(Hand.kRight) && operator.getPOV() == 0);
  }

  public boolean cargoShipTop(){
    return (operator.getPOV() == 0);
  }

  public boolean backArmFloorPressed(){
    return (operator.getPOV() == 270);
  }

  public boolean backArmLowPressed(){
    return (operator.getBumper(Hand.kLeft) && operator.getPOV() == 270);
  }

  public boolean backRocketCenter(){
    return (operator.getBumper(Hand.kRight) && operator.getPOV() == 180);
  }

  public boolean backCargoShipTop(){
    return (operator.getPOV() == 180);
  }

  public boolean isCameraSwitchPressed(){
    return pilot.getBumper(Hand.kLeft); // button tbd
  }

  public double moveArm(){
    if (Math.abs(operator.getY(Hand.kLeft)) < 0.20)
      return 0;
    else
      return operator.getY(Hand.kLeft);
  }

}
