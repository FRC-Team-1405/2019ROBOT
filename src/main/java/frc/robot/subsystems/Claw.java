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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.RobotMap;
import frc.robot.commands.ArmController;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  
  private WPI_TalonSRX talon = new WPI_TalonSRX(RobotMap.clawTalon);
  private DoubleSolenoid solenoid = new DoubleSolenoid(3, 6);
  
  public Claw(){
    talon.setName("Intake");
    this.addChild(talon);
    solenoid.setName("Solenoid");
    this.addChild(solenoid);
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
}
