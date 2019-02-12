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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.ArmController;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  
  private WPI_TalonSRX pivotTalon = new WPI_TalonSRX(RobotMap.pivotTalon);
  private TalonSRX pivotTalonSlave = new TalonSRX(RobotMap.pivotTalonSlave);
  
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

        configureTalon(pivotTalon);
    pivotTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    pivotTalon.configNeutralDeadband(0.001, 10);
    configureTalon(pivotTalonSlave);

    pivotTalon.set(ControlMode.PercentOutput, 0);
    pivotTalonSlave.set(ControlMode.Follower, RobotMap.pivotTalon);
    
    pivotTalon.setName("Pivot Arm"); 
    this.addChild(pivotTalon); 
    LiveWindow.add(pivotTalon);

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

  public void configureTalon(TalonSRX talonSRX){
    talonSRX.configPeakCurrentDuration(50, 10);
    talonSRX.configPeakCurrentLimit(40, 10);
    talonSRX.configContinuousCurrentLimit(35, 10);
    talonSRX.enableCurrentLimit(true);
    talonSRX.configNeutralDeadband(0.001, 10);
  }

  public double getArmPosition(){
   return pivotTalon.getSensorCollection().getAnalogInRaw();
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

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Current A", () -> { return pivotTalon.getOutputCurrent(); }, null );
    builder.addDoubleProperty("Current B", () -> { return pivotTalonSlave.getOutputCurrent(); }, null );
    builder.addDoubleProperty("Arm T P", () -> { return pivotTalon.getActiveTrajectoryPosition();}, null);
    builder.addDoubleProperty("Arm T V", () -> { return pivotTalon.getActiveTrajectoryVelocity();}, null);
    builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
  }
}
