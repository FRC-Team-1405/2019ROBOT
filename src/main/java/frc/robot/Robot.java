/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel; 
import edu.wpi.first.wpilibj.CameraServer; 
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveBaseController;
import frc.robot.commands.DriveToVisionTarget;
import frc.robot.commands.FloorHatch;
import frc.robot.commands.LoadHatch;
import frc.robot.commands.PlaceHatch;
import frc.robot.lib.LidarReader;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.GyroSystem;
import frc.robot.subsystems.SoloStilts;
import frc.robot.subsystems.Vision;
//import edu.wpi.first.wpilibj.livewindow.*;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static ArcadeDrive driveBase;
  public static Claw claw;
  public static Arm arm;
  public static Vision vision;
  public static GyroSystem gyro;
  public static SoloStilts stilts;

  Command autonomousCommand;
  Command teleopCommand;
  public static Command driveToVisionTarget;
  public static Command loadHatch;
  public static Command floorHatch;
  public static Command placeHatch;

  public static LidarReader lidarReader;
  public static boolean limitDebug = false;

  private void initDebugFlag(){
    limitDebug = !"DEBUG".equals( DriverStation.getInstance().getGameSpecificMessage() );
  }
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    driveBase = new ArcadeDrive();
    vision = new Vision();
    claw = new Claw();
    arm = new Arm();  
    gyro = new GyroSystem();
    stilts = new SoloStilts(); 

    // removed until lidar is finished
    // lidarReader = new LidarReader();
    // lidarReader.start();

    //claw = new Claw();
    // chooser.addOption("My Auto", new MyAutoCommand());
//    LiveWindow.add(claw);

    SmartDashboard.putData( new PowerDistributionPanel() );

    autonomousCommand = new DriveBaseController();
    teleopCommand = autonomousCommand;
    driveToVisionTarget = new DriveToVisionTarget();
    loadHatch = new LoadHatch();
    floorHatch = new FloorHatch();
    placeHatch = new PlaceHatch();

    SmartDashboard.putData("Drive Base", driveBase);
    SmartDashboard.putData("DriveToVisionTarget", driveToVisionTarget);
    SmartDashboard.putData("Gyro System", gyro);
    SmartDashboard.putData("Vision System", vision);

    //test comment for new pc github setup

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    arm.adjustArmPosition(0.0);
    // lidarReader.killThread();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    arm.adjustArmPosition(0.0);
    vision.setCameraMode();
    m_oi.OnRobotEnable();
    autonomousCommand.start();
    initDebugFlag();  
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
//    arm.adjustArmPosition(0.0);
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    initDebugFlag();

    if(DriverStation.getInstance().isFMSAttached() == false){
      arm.adjustArmPosition(0.0);
    }
    
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    //claw.closeClawTop();
    //claw.closeClawBottom();
    m_oi.OnRobotEnable();

    teleopCommand.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }


  @Override
  public void testInit() {
    initDebugFlag();
    arm.adjustArmPosition(0.0);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
