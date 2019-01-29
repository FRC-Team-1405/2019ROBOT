/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ArmController;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  
  private WPI_TalonSRX pivotTalon = new WPI_TalonSRX(RobotMap.pivotTalon);

  private static double kP = 0.0;
  private static double kI = 0.0;
  private static double kD = 0.0;
  private static final String keyP = "Arm_P";
  private static final String keyI = "Arm_I";
  private static final String keyD = "Arm_D";

  private static double floorPos = 0.0;
  private static double lowPos = 0.0;
  private static final String keyFloorPos = "Arm_FloorPosition";
  private static final String keyLowPos = "Arm_EjectPositionLow";

  public Arm() {
    // Intert a subsystem name and PID values here
    super("Arm", kP, kI, kD);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.

    pivotTalon.setName("Pivot Arm"); 
    this.addChild(pivotTalon); 

    Preferences prefs = Preferences.getInstance(); 
    if (!prefs.containsKey(keyP)) {
        prefs.putDouble("Arm P", kP);
    }
    if (!prefs.containsKey(keyI)) {
        prefs.putDouble("Arm I", kI);
    }
    if (!prefs.containsKey(keyD)) {
      prefs.putDouble("Arm D", kD);
    }

    if (!prefs.containsKey(keyFloorPos)) {
      prefs.putDouble("Floor Position", floorPos);
    }
    if (!prefs.containsKey(keyLowPos)) {
      prefs.putDouble("Eject Position Low", floorPos);
    }
    

    kP = prefs.getDouble(keyP, kP);
    kI = prefs.getDouble(keyI, kI); 
    kD = prefs.getDouble(keyD, kD); 

    floorPos = prefs.getDouble(keyFloorPos, floorPos);
    lowPos = prefs.getDouble(keyLowPos, lowPos);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmController());
  }

  public void floor(){
    setSetpoint(floorPos);
  }

  public void low(){
    setSetpoint(lowPos);
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return pivotTalon.getSelectedSensorPosition();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    pivotTalon.set(output); 
  }
}
