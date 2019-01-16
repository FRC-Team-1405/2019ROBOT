/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.RobotMap;
import frc.robot.commands.Intake;

/**
 * Add your docs here.
 */
public class Claw extends Subsystem {
  WPI_TalonSRX talon = new WPI_TalonSRX(RobotMap.clawTalon);
  public Claw(){
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new Intake());
  }
}
