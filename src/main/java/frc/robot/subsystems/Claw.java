/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotMap;
import frc.robot.commands.ArmController;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  
  private WPI_TalonSRX intakeTalonA = new WPI_TalonSRX(RobotMap.clawTalonA);
  private WPI_TalonSRX intakeTalonB = new WPI_TalonSRX(RobotMap.clawTalonB);
  private DoubleSolenoid solenoid = new DoubleSolenoid(3, 6);
  private WPI_TalonSRX pivotTalon = new WPI_TalonSRX(RobotMap.pivotTalon);

  public Claw(){
    intakeTalonA.setName("Intake");
    this.addChild(intakeTalonA);
    intakeTalonB.setName("Intake 2"); 
    this.addChild(intakeTalonB); 
    solenoid.setName("Solenoid");
    this.addChild(solenoid);
    pivotTalon.setName("Pivot Arm"); 
    this.addChild(pivotTalon); 
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmController());
  }

  public void openClaw() {
    solenoid.set(Value.kForward);
  }

  public void closeClaw() {
    solenoid.set(Value.kReverse);
  }

  public void pivot(int speed){
    pivotTalon.set(speed); 
  } 

  public void intakeCargo(double speed) { 
    intakeTalonA.set(speed); 
    intakeTalonB.set(speed); 
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Pivot Current", this.pivotTalon::getOutputCurrent, null );
  }

}
