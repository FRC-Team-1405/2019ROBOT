/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.ClawController;
import frc.robot.lib.ExtendedTalon;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  
  private WPI_TalonSRX intakeTalonA = new WPI_TalonSRX(RobotMap.clawTalonA);
  private WPI_TalonSRX intakeTalonB = new WPI_TalonSRX(RobotMap.clawTalonB);
  private DoubleSolenoid solenoidTop = new DoubleSolenoid(1, 2);
  private DoubleSolenoid 
  solenoidBottom = new DoubleSolenoid(3, 4);
  private PIDSource currentSource = new PIDSource(){
    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {}
    @Override
    public double pidGet() {
      return ((intakeTalonA.getOutputCurrent()+intakeTalonB.getOutputCurrent())/2);
    }
    @Override
    public PIDSourceType getPIDSourceType() { return PIDSourceType.kDisplacement; }
  };
  private LinearDigitalFilter currentFilter = LinearDigitalFilter.movingAverage(currentSource, 10);

  private static double intakeSpeed = 1.0;
  private static double outputSpeed = 1.0;
  private static final double cargoAcquiredCurrent = 1.0; //TBD
  private static final double hatchAcquiredCurrent = 1.0; //TBD
  private static final String keyIntake = "Claw_Intake_Speed";
  private static final String keyOutput = "Claw_Output_Speed";

  public Claw(){
    ExtendedTalon.configCurrentLimit(intakeTalonA);
    intakeTalonA.setName("Intake A");
    this.addChild(intakeTalonA);
    ExtendedTalon.configCurrentLimit(intakeTalonB);
    intakeTalonB.setName("Intake B"); 
    this.addChild(intakeTalonB); 
    solenoidTop.setName("Solenoid Front");
    this.addChild(solenoidTop);
    solenoidBottom.setName("Solenoid Back");
    this.addChild(solenoidBottom);

    Preferences prefs = Preferences.getInstance(); 
    if (!prefs.containsKey(keyIntake)) {
        prefs.putDouble("Intake Speed", intakeSpeed);
    }
    if (!prefs.containsKey(keyOutput)) {
        prefs.putDouble("Output Speed", outputSpeed);
    }

    intakeSpeed = prefs.getDouble(keyIntake, intakeSpeed);
    outputSpeed = prefs.getDouble(keyOutput, outputSpeed); 

    System.out.printf("%s %f %s %f\n", keyIntake, intakeSpeed, keyOutput, outputSpeed);

    this.openClawTop();
    this.openClawBottom();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ClawController());
  }

  public void openClawTop() {
    solenoidTop.set(Value.kForward);
  }

  public void closeClawTop() {
    solenoidTop.set(Value.kReverse);
  }

  public void openClawBottom(){
    solenoidBottom.set(Value.kForward);
    stopHatch();
  }

  public void closeClawBottom(){
    solenoidBottom.set(Value.kReverse);
    stopHatch();  
  }

  public void intakeCargo(double speed) { 
    intakeTalonA.set(speed); 
    intakeTalonB.set(speed); 
  }

  public void releaseCargo(double speed){
    intakeTalonA.set(-speed);
    intakeTalonB.set(-speed);
  }

  public void stopCargo(){
    intakeTalonA.set(0);
    intakeTalonB.set(0);
  }


  public void intakeHatch(double speed) {
    intakeTalonA.set(speed);
    intakeTalonB.set(speed);
  }
  public void releaseHatch(double speed) {
    intakeTalonA.set(-speed);
    intakeTalonB.set(-speed);
  }
  public void stopHatch() {
    intakeTalonA.set(0);
    intakeTalonB.set(0);
  }
  public void holdHatch(){
    intakeTalonA.set(0.25);
    intakeTalonB.set(0.25);
  }

  public boolean isCargoAcquired(){
    return (currentFilter.get() > cargoAcquiredCurrent);
  }

  public boolean isHatchAcquired(){
    return (currentFilter.get() > hatchAcquiredCurrent);
  }
  
  @Override
  public void periodic() {
    currentFilter.pidGet();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Current", () -> {return currentFilter.get();}, null);
    builder.addBooleanProperty("Top Claw", () -> {return solenoidTop.get() ==  Value.kForward;}, null);
    builder.addBooleanProperty("Bottom Claw", () -> {return solenoidBottom.get() ==  Value.kForward;}, null);
  }

}
