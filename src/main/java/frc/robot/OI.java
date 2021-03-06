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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public XboxController pilot = new XboxController(RobotMap.pilot);
  public XboxController operator = new XboxController(RobotMap.operator);
  public XboxController testJoystick = new XboxController(RobotMap.testJoystick);
  boolean guitarMode = false;
  private static final String keyGuitarMode = "OI_GuitarMode";

  public OI(){
    SmartDashboard.putBoolean(keyGuitarMode, guitarMode);
  }

  public void OnRobotEnable(){
    guitarMode = SmartDashboard.getBoolean(keyGuitarMode, guitarMode);
  }
  
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
  
  public double extendStilts(){
    return pilot.getTriggerAxis(Hand.kRight);
  }

  public double retractStilts(){
    return pilot.getTriggerAxis(Hand.kLeft);
  }

  public boolean isDriveToVisionTargetPressed(){
    return pilot.getStickButton(Hand.kRight);//button tbd
  }
  
  public boolean isDriveToVisionTargetReleased(){
    return pilot.getStickButtonReleased(Hand.kRight);//button tbd
  }

  public boolean isDriveToDistanceEnabled(){
    return false;//button tbd
  }

  public boolean armFloorPressed(){
    if(guitarMode){
      return(operator.getAButton() && operator.getPOV() == 0);
    }
    return (operator.getPOV() == 90);
  }

  public boolean backArmFloorPressed(){
    if(guitarMode){
      return(operator.getAButton() && operator.getPOV() == 180);
    }
    return (operator.getPOV() == 270);
  }

  public boolean cargoShipTop(){
    if(guitarMode){
      return(operator.getYButton() && operator.getPOV() == 0);
    }
    return (operator.getPOV() == 0);
  }

  public boolean backCargoShipTop(){
    if(guitarMode){
      return(operator.getYButton() && operator.getPOV() == 180);
    }
    return (operator.getPOV() == 180);
  }

  public boolean rocketCenter(){
    if(guitarMode){
      return(operator.getXButton() && operator.getPOV() == 0);
    }
    return (operator.getBumper(Hand.kRight) && operator.getPOV() != -1 && (operator.getPOV() > 270 || operator.getPOV() < 90));
  }

  public boolean backRocketCenter(){
    if(guitarMode){
      return(operator.getXButton() && operator.getPOV() == 180);
    }
    return (operator.getBumper(Hand.kRight) && operator.getPOV() < 270 && operator.getPOV() > 90);
  }

  public boolean armHatchPressed(){
    if(guitarMode){
      return(operator.getBButton() && operator.getPOV() == 0);
    }
    return (operator.getBumper(Hand.kLeft) && operator.getPOV() < 180 && operator.getPOV() > 0);
  }

  public boolean backArmHatchPressed(){
    if(guitarMode){
      return(operator.getBButton() && operator.getPOV() == 180);
    }
    return (operator.getBumper(Hand.kLeft) && operator.getPOV() > 180);
  }

  public boolean isCameraSwitchPressed(){
    return pilot.getYButtonPressed();
  }

  public boolean isLoadHatchPressed(){
    if(guitarMode){
      operator.getBumper(Hand.kLeft);
    }
    return operator.getBButtonPressed();
  }

  public boolean isLoadHatchReleased(){
    if(guitarMode){
      return !operator.getBumper(Hand.kLeft);
    }
    return !operator.getBButton();
  }

  public boolean cancelCommand(){
    return operator.getBackButtonPressed();
  }

  public boolean manualArmControEnabled(){
    return operator.getStickButtonPressed(Hand.kRight);
  }

  public boolean manualArmControlDisabled(){
    return operator.getStickButtonReleased(Hand.kRight);
  }

  public double manualArmControl(){
    if(guitarMode){
      if(operator.getPOV() > 180 && operator.getPOV() < 360){
        return 0.75;
      }else if(operator.getPOV() > 0 && operator.getPOV() < 180){
        return -0.75;
      }else if(operator.getBackButton()){
        return 0.3;
      }else if(operator.getStartButton()){
        return -0.3;
      }
      return 0;
    }
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

  public boolean isLoadCargoPressed() {
    return false;
  }
  public boolean isLoadCargoReleased() {
    return false;
  }

  boolean lastDecreasePOV = false;
  public boolean decreaseClawPower(){
    if (pilot.getPOV() == -1){
      lastDecreasePOV = false;
      return false;
    }
    boolean decrease = (pilot.getPOV() < 270 && pilot.getPOV() > 90);
    if (decrease == lastDecreasePOV){
      return false;
    }
    lastDecreasePOV = decrease;
    return lastDecreasePOV;
  }

  boolean lastIncreasePOV = false;
  public boolean increaseClawPower(){
    if (pilot.getPOV() == -1){
      lastIncreasePOV = false;
      return false;
    }
    boolean increase = (pilot.getPOV() > 270 || pilot.getPOV() < 90);
    if (increase == lastIncreasePOV){
      return false;
    }
    lastIncreasePOV = increase;
    return lastIncreasePOV;
  }

  public void rumbleVision(boolean left, boolean right){
    pilot.setRumble(RumbleType.kRightRumble , right ? 1.0 : 0.0);
    pilot.setRumble(RumbleType.kLeftRumble , left ? 1.0 : 0.0);
  }
}
