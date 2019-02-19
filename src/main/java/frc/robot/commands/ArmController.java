/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmController extends Command {

  public enum ArmPosition{
    FLOOR_FRONT, FLOOR_BACK, CARGO_SHIP_FRONT, CARGO_SHIP_BACK,
    HATCH_FRONT, HATCH_BACK, ROCKET_FRONT, ROCKET_BACK, UNKNOWN;
  }

  public ArmController() {
    requires(Robot.arm);
  }

  private ArmPosition armPosition = ArmPosition.UNKNOWN;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.isFloorHatchPressed()){
      Robot.floorHatch.start();
    }
    if(Robot.m_oi.isLoadHatchPressed()){
      Robot.loadHatch.start();
    }
    if(Robot.m_oi.isPlaceHatchPressed()){
      Robot.placeHatch.start();
    }

    if(Robot.m_oi.rocketCenter()){
      Robot.arm.frontRocketCenter();
    } else if(Robot.m_oi.cargoShipTop()){
      Robot.arm.frontCargoShipTop();
    } else if(Robot.m_oi.backRocketCenter()){
      Robot.arm.backRocketCenter();
    } else if(Robot.m_oi.backCargoShipTop()){
      Robot.arm.backCargoShipTop();
    }else if(Robot.m_oi.armFloorPressed()) {
      Robot.arm.frontFloor();
    } else if(Robot.m_oi.armLowPressed()){
      Robot.arm.frontLow();
    } else if(Robot.m_oi.backArmFloorPressed()) {
      Robot.arm.backFloor();
    } else if(Robot.m_oi.backArmLowPressed()){
      Robot.arm.backLow();
    } 

    if (Robot.m_oi.manualArmControEnabled()) {
      Robot.arm.adjustArmPosition(Robot.m_oi.manualArmControl());
    }
    if (Robot.m_oi.manualArmControlDisabled()) {
      Robot.arm.adjustArmPosition(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
