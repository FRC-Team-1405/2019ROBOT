/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public XboxController pilot = new XboxController(RobotMap.pilot);
  public XboxController operator = new XboxController(RobotMap.operator);
  public XboxController testJoystick = new XboxController(RobotMap.testJoystick);
  
  public boolean cargoOutputPressed() {
    return pilot.getBumper(Hand.kLeft);
  }
  public boolean cargoIntakePressed() {
    return pilot.getBumper(Hand.kRight);
  }

  public boolean  topClawOpenPressed(){
    return pilot.getAButtonPressed();
  }

  public boolean bottomClawOpenPressed(){
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
    return false;//button tbd
  }
  
  public boolean isDriveToDistanceEnabled(){
    return false;//button tbd
  }

  public boolean armFloorPressed(){
    return (operator.getPOV() == 90);
  }

  public boolean backArmFloorPressed(){
    return (operator.getPOV() == 270);
  }

  public boolean cargoShipTop(){
    return (operator.getPOV() == 0);
  }

  public boolean backCargoShipTop(){
    return (operator.getPOV() == 180);
  }

  public boolean rocketCenter(){
    return (operator.getBumper(Hand.kRight) && operator.getPOV() != -1 && (operator.getPOV() > 270 || operator.getPOV() < 90));
  }

  public boolean backRocketCenter(){
    return (operator.getBumper(Hand.kRight) && operator.getPOV() < 270 && operator.getPOV() > 90);
  }

  public boolean armLowPressed(){
    return (operator.getBumper(Hand.kLeft) && operator.getPOV() < 180 && operator.getPOV() > 0);
  }

  public boolean backArmLowPressed(){
    return (operator.getBumper(Hand.kLeft) && operator.getPOV() > 180);
  }

  public boolean isCameraSwitchPressed(){
    return false; // button tbd
  }

  public boolean isLoadHatchPressed(){
    return operator.getBButtonPressed();
  }

  public boolean isLoadHatchReleased(){
    return !operator.getBButton();
  }

  public boolean cancelCommand(){
    return operator.getBackButtonPressed();
  }

  public boolean manualArmControEnabled(){
    return operator.getStickButton(Hand.kRight);
  }

  public boolean manualArmControlDisabled(){
    return operator.getStickButtonReleased(Hand.kRight);
  }

  public double manualArmControl(){
    return operator.getY(Hand.kRight);
  }

  public boolean isFloorHatchPressed(){
    return operator.getAButtonPressed();
  }

  public boolean isFloorHatchReleased(){
    return !operator.getAButton();
  }

  public boolean isPlaceHatchPressed(){
    return operator.getYButtonPressed();
  }

  public boolean isPlaceHatchReleased(){
    return !operator.getYButton();
  }

  public double extendStilts(){
    return pilot.getTriggerAxis(Hand.kRight);
  }

  public double retractStilts(){
    return pilot.getTriggerAxis(Hand.kLeft);
  }

  public boolean isLoadCargoPressed() {
    return false;
  }
  public boolean isLoadCargoReleased() {
    return false;
  }
}
