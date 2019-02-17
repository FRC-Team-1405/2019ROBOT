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
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.ArmController;
import frc.robot.lib.ExtendedTalon;
import frc.robot.lib.TalonPID;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  
  private WPI_TalonSRX pivotTalon = new WPI_TalonSRX(RobotMap.pivotTalon);
  private TalonPID armPID =  new TalonPID(pivotTalon, ControlMode.Position);

  // private static double kP = 0.0;
  // private static double kI = 0.0;
  // private static double kD = 0.0;
  // moving Talon prefs to talon
  // private static final String keyP = "Arm_P";
  // private static final String keyI = "Arm_I";
  // private static final String keyD = "Arm_D";

  private static double floorPickup = 0.0;
  private static double lowScoring = 0.0;
  private static double rocketCenterCargo = 0.0;
  private static double cargoShipCargo = 0.0;
  private static final String keyFloorPickup = "Arm_FloorPosition";
  private static final String keyLowScoring = "Arm_EjectPositionLow"; 
  private static final String keyRocketCenterCargo = "Arm_EjectCenterRocket";
  private static final String keyCargoShipCargo = "Arm_EjectCargoShipCargo";

  public Arm() {
    configureTalon(pivotTalon);
    pivotTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    pivotTalon.configNeutralDeadband(0.001, 10);

    pivotTalon.set(ControlMode.PercentOutput, 0);
    
    pivotTalon.setName("Pivot Arm");
    this.addChild(pivotTalon); 
    LiveWindow.add(pivotTalon);

    armPID.setName("Pivot PID");
    this.addChild(armPID);
    LiveWindow.add(armPID);

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

    floorPickup = prefs.getDouble(keyFloorPickup, floorPickup);
    lowScoring = prefs.getDouble(keyLowScoring, lowScoring);
    rocketCenterCargo = prefs.getDouble(keyRocketCenterCargo, rocketCenterCargo);
    cargoShipCargo = prefs.getDouble(keyCargoShipCargo, cargoShipCargo);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmController());
  }

  public void floor(){
    pivotTalon.set(ControlMode.Position, floorPickup);
  }

  public void low(){
    pivotTalon.set(ControlMode.Position, lowScoring);
  }

  public void rocketCenter(){
    pivotTalon.set(ControlMode.Position, rocketCenterCargo);
  }

  public void cargoShipTop(){
    pivotTalon.set(ControlMode.Position, cargoShipCargo);
  }

  public void adjustArmPosition(double position){
    position = pivotTalon.getSelectedSensorPosition() + (position * 10.0) ;
  }

  public void configureTalon(TalonSRX talonSRX){
    ExtendedTalon.configCurrentLimit(talonSRX);
    talonSRX.configNeutralDeadband(0.001, 10);
    talonSRX.setSensorPhase(true);
  }

  public double getArmPosition(){
   return pivotTalon.getSensorCollection().getAnalogInRaw();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Current A", () -> { return pivotTalon.getOutputCurrent(); }, null );
    builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
    builder.addDoubleProperty("PID Target", () -> { return pivotTalon.getClosedLoopTarget(0);}, null);
    builder.addDoubleProperty("PID Error", () -> { return pivotTalon.getClosedLoopError(0);}, null);
    builder.addDoubleProperty("PID Position", () -> { return pivotTalon.getSelectedSensorPosition(0);}, null);


    
  }
}
