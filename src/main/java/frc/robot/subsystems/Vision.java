/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import org.opencv.core.Mat;
//import org.opencv.imgproc.Imgproc;

//import edu.wpi.cscore.CvSink;
//import edu.wpi.cscore.CvSource;
//import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.Limelight;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /* // Add these variables back in when it's time.
  private static final int WIDTH = 640;
  private static final int HEIGHT = 480;
  */
  Limelight front = new Limelight("front"); 
  Limelight back = new Limelight("back");
  Limelight selected = back;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void initCamera(){
    CameraServer.getInstance().startAutomaticCapture();
  }

  public void setLineTarget(){
    // setup Limelight pipeline
    selected.setPipeline((byte)0);  
  }

  public double getLineTarget(){
  return selected.getTX(); 
     
  }

  public boolean isTargetAcquired(){
    return selected.hasTarget();
  }

  public void setFront(){
    selected = front;
  }

  public void setBack(){
    selected = back;
  }

  public void toggleCamera(){
    selected = (selected == front) ? back : front;
    SmartDashboard.putBoolean("Camera Switch", selected == front);
  }
}
