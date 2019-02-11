/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.ClawController;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  
  private WPI_TalonSRX intakeTalonA = new WPI_TalonSRX(RobotMap.clawTalonA);
  private WPI_TalonSRX intakeTalonB = new WPI_TalonSRX(RobotMap.clawTalonB);
  private DoubleSolenoid solenoid ;//= new DoubleSolenoid(3, 6);
  
  private static double intakeSpeed = 1.0;
  private static double outputSpeed = 1.0;
  private static final String keyIntake = "Claw_Intake_Speed";
  private static final String keyOutput = "Claw_Output_Speed";

  public Claw(){
    intakeTalonA.setName("Intake A");
    this.addChild(intakeTalonA);
    intakeTalonB.setName("Intake B"); 
    this.addChild(intakeTalonB); 
//    solenoid.setName("Solenoid");
//    this.addChild(solenoid);

    
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

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ClawController());
  }

  public void openClaw() {
    solenoid.set(Value.kForward);
  }

  public void closeClaw() {
    solenoid.set(Value.kReverse);
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
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

}
