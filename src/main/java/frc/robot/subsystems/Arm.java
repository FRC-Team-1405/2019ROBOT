/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArmController;
import frc.robot.commands.ArmController.ArmPosition;
import frc.robot.lib.ExtendedTalon;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  
  private WPI_TalonSRX pivotTalon = new WPI_TalonSRX(RobotMap.pivotTalon);

  private static double floorPickup = 36.0;
  private static double lowScoring = 126.0;
  private static double rocketCenterCargo = 322.0;
  private static double cargoShipCargo = 322.0;
  private static double backFloorPickup = 927.0;
  private static double backLowScoring = 821.0;
  private static double backRocketCenterCargo = 537.0;
  private static double backCargoShipCargo = 537.0;
  private static double maxArmError = 25.0;
  
  private static final String keyFloorPickup = "Arm_FloorPosition";
  private static final String keyLowScoring = "Arm_EjectPositionLow"; 
  private static final String keyRocketCenterCargo = "Arm_EjectCenterRocket";
  private static final String keyCargoShipCargo = "Arm_EjectCargoShipCargo";
  private static final String keyBackFloorPickup = "Arm_BackFloorPosition";
  private static final String keyBackLowScoring = "Arm_BackEjectPositionLow"; 
  private static final String keyBackRocketCenterCargo = "Arm_BackEjectCenterRocket";
  private static final String keyBackCargoShipCargo = "Arm_BackEjectCargoShipCargo";

  private static final int ARM_MOVE_DELAY_MS = 150; 

  private ArmPosition armPosition = ArmPosition.UNKNOWN;
  private long armPositionSetTime = 0;

  public Arm() {
    configureTalon(pivotTalon);
    pivotTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    pivotTalon.configNeutralDeadband(0.001, 10);

    pivotTalon.set(ControlMode.PercentOutput, 0);
    
    pivotTalon.setName("Pivot Arm");
    this.addChild(pivotTalon); 
    LiveWindow.add(pivotTalon);

    Preferences prefs = Preferences.getInstance(); 

    if (!prefs.containsKey(keyFloorPickup)) {
      prefs.putDouble(keyFloorPickup, floorPickup);
    }
    if (!prefs.containsKey(keyLowScoring)) {
      prefs.putDouble(keyLowScoring, lowScoring);
    }

    if (!prefs.containsKey(keyRocketCenterCargo)){
      prefs.putDouble(keyRocketCenterCargo, rocketCenterCargo);
    }

    if(!prefs.containsKey(keyCargoShipCargo)){
      prefs.putDouble(keyCargoShipCargo, cargoShipCargo);
    }

    if (!prefs.containsKey(keyBackFloorPickup)) {
      prefs.putDouble(keyBackFloorPickup, backFloorPickup);
    }
    if (!prefs.containsKey(keyBackLowScoring)) {
      prefs.putDouble(keyBackLowScoring, backLowScoring);
    }

    if (!prefs.containsKey(keyBackRocketCenterCargo)){
      prefs.putDouble(keyBackRocketCenterCargo, backRocketCenterCargo);
    }

    if(!prefs.containsKey(keyBackCargoShipCargo)){
      prefs.putDouble(keyBackCargoShipCargo, backCargoShipCargo);
    }

    floorPickup = prefs.getDouble(keyFloorPickup, floorPickup);
    lowScoring = prefs.getDouble(keyLowScoring, lowScoring);
    rocketCenterCargo = prefs.getDouble(keyRocketCenterCargo, rocketCenterCargo);
    cargoShipCargo = prefs.getDouble(keyCargoShipCargo, cargoShipCargo);
    backFloorPickup = prefs.getDouble(keyBackFloorPickup, backFloorPickup);
    backLowScoring = prefs.getDouble(keyBackLowScoring, backLowScoring);
    backRocketCenterCargo = prefs.getDouble(keyBackRocketCenterCargo, backRocketCenterCargo);
    backCargoShipCargo = prefs.getDouble(keyBackCargoShipCargo, backCargoShipCargo);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmController());
  }

  public void frontFloor(){
    frontFloor(true);
  }
  public void frontFloor(boolean openClaws){
    if (openClaws){
      Robot.claw.openClawTop();
      Robot.claw.openClawBottom();
    }
    pivotTalon.set(ControlMode.Position, floorPickup);
    armPosition = ArmPosition.FLOOR_FRONT;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void frontHatch(){
    pivotTalon.set(ControlMode.Position, lowScoring);
    armPosition = ArmPosition.HATCH_FRONT;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void frontRocketCenter(){
    pivotTalon.set(ControlMode.Position, rocketCenterCargo);
    armPosition = ArmPosition.ROCKET_FRONT;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void frontCargoShipTop(){
    pivotTalon.set(ControlMode.Position, cargoShipCargo);
    armPosition = ArmPosition.CARGO_SHIP_FRONT;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void backFloor(){
    backFloor(true);
  }
  public void backFloor(boolean openClaws){
    if (openClaws){
      Robot.claw.openClawTop();
      Robot.claw.openClawBottom();
    }
    pivotTalon.set(ControlMode.Position, backFloorPickup);
    armPosition = ArmPosition.FLOOR_BACK;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void backHatch(){
    pivotTalon.set(ControlMode.Position, backLowScoring);
    armPosition = ArmPosition.HATCH_BACK;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void backRocketCenter(){
    pivotTalon.set(ControlMode.Position, backRocketCenterCargo);
    armPosition = ArmPosition.ROCKET_BACK;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void backCargoShipTop(){
    pivotTalon.set(ControlMode.Position, backCargoShipCargo);
    armPosition = ArmPosition.CARGO_SHIP_BACK;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void adjustArmPosition(double position){
    pivotTalon.set(position);
    armPosition = ArmPosition.UNKNOWN;
    armPositionSetTime = System.currentTimeMillis();
  }

  public void adjustSetPoint(double delta){
    if (pivotTalon.getControlMode() != ControlMode.Position) {
      return ;
    }
    
    pivotTalon.set(ControlMode.Position, pivotTalon.getClosedLoopTarget()+delta);
    armPositionSetTime = System.currentTimeMillis();
  }

  public void configureTalon(TalonSRX talonSRX){
    ExtendedTalon.configCurrentLimit(talonSRX);
    talonSRX.configNeutralDeadband(0.001, 10);
    talonSRX.setSensorPhase(true);
  }

  public double getArmPosition(){
   return pivotTalon.getSelectedSensorPosition(0);
  }

  public ArmPosition armInPosition(){
    if (pivotTalon.getControlMode() != ControlMode.Position) {
      return armPosition ;
    }

    return (Math.abs(pivotTalon.getClosedLoopError()) < maxArmError) && (System.currentTimeMillis() > armPositionSetTime+ARM_MOVE_DELAY_MS) 
                ? armPosition
                : ArmPosition.UNKNOWN;
  }

  public double getCargoShipCargoPos(boolean inBack) {
    return inBack ? backCargoShipCargo : cargoShipCargo;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    if(Robot.limitDebug)
       return;

    // Shuffleboard Debug
    builder.addDoubleProperty("Current A", () -> { return pivotTalon.getOutputCurrent(); }, null );
    builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
    builder.addDoubleProperty("PID Target", () -> {
      if (pivotTalon.getControlMode() != ControlMode.Position) {
        return 0 ;
      }
      return pivotTalon.getClosedLoopTarget(0);
      }, null);
    builder.addDoubleProperty("PID Error", () -> {
      if (pivotTalon.getControlMode() != ControlMode.Position) {
        return 0 ;
      }
      return pivotTalon.getClosedLoopError(0);
    }, null);
    builder.addDoubleProperty("PID Position", () -> { return pivotTalon.getSelectedSensorPosition(0);}, null);
  }
}
