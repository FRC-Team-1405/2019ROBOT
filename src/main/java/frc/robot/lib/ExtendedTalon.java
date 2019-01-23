/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 * Wrapper for TalonSRX, makes talon current based, not percentvoltage
 */
public class ExtendedTalon extends WPI_TalonSRX { 
  
    public ExtendedTalon(int deviceNumber) {
		super(deviceNumber);
	}

    private static double currentMin = -40.0;
    private static double currentMax = 40.0;
    private static double inputMin = -1.0;
    private static double inputMax = 1.0;
    private static double slope = (currentMax - currentMin) / (inputMax - inputMin);
    @Override
    public void set(double speed) {
        double output = currentMin + slope * (speed - inputMin);
        super.set(ControlMode.Current, output);
    }
}
