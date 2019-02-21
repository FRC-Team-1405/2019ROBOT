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
  public enum Piece {CARGO, HATCH};
  private Piece currentManipulatedPiece = Piece.HATCH;

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
    return (operator.getBumper(Hand.kRight) && (operator.getPOV() > 270 || operator.getPOV() < 90));
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
    if(currentManipulatedPiece == Piece.HATCH) {
      return operator.getBButtonPressed();
    } else { return false; }
  }

  public boolean isLoadHatchReleased(){
    if(currentManipulatedPiece == Piece.HATCH) {
      return operator.getBButtonReleased();
    } else { return false; }
  }


  public boolean isLoadCargoPressed() {
    if(currentManipulatedPiece == Piece.CARGO) {
      return operator.getBButtonPressed();
    } else { return false; }
  }

  public boolean isLoadCargoReleased() {
    if(currentManipulatedPiece == Piece.CARGO) {
      return operator.getBButtonReleased();
    } else { return false; }
  }


  public boolean switchPieceType() {
    return operator.getXButtonPressed();
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
    return operator.getAButtonReleased();
  }

  public boolean isPlaceHatchPressed(){
    return operator.getYButton();
  }

  public boolean isPlaceHatchReleased(){
    return operator.getYButtonReleased();
  }

  public double extendStilts(){
    return pilot.getTriggerAxis(Hand.kRight);
  }

  public double retractStilts(){
    return pilot.getTriggerAxis(Hand.kLeft);
  }

  public Piece getCurrentManipulatedPiece() {
    return currentManipulatedPiece;
  }
  public void setCurrentManipulatedPiece(Piece newPiece) {
    currentManipulatedPiece = newPiece;
  }
}
