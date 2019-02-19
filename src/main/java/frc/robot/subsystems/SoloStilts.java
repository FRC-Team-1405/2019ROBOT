/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SoloStiltsController;

/**
 * Add your docs here.
 */
public class SoloStilts extends Subsystem {

  private TalonSRX soloStiltTalon = new TalonSRX(RobotMap.soloStiltTalon); 
  private TalonSRX soloStiltTalon2 = new TalonSRX(RobotMap.soloStiltTalon2);
  private TalonSRX soloStiltTalon3 = new TalonSRX(RobotMap.soloStiltTalon3);
  private TalonSRX soloStiltTalon4 = new TalonSRX(RobotMap.soloStiltTalon4);


  public void moveStilts(double speed){
    soloStiltTalon.set(ControlMode.PercentOutput, speed);
    soloStiltTalon2.set(ControlMode.PercentOutput, speed);
    soloStiltTalon3.set(ControlMode.PercentOutput, speed);
    soloStiltTalon4.set(ControlMode.PercentOutput, speed);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new SoloStiltsController());
  
  }
}
