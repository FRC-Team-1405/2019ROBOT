/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;


/**
 * Wrapper for TalonSRX, makes talon current based, not percentvoltage
 */
public class ExtendedTalon extends WPI_TalonSRX { 
    public ExtendedTalon(int deviceNumber) {
        super(deviceNumber);
        
        String keyMin = String.format("Talon_%d_Current_Min", deviceNumber); 
        String keyMax = String.format("Talon_%d_Current_Max", deviceNumber); 
        
        Preferences prefs = Preferences.getInstance(); 
        if (!prefs.containsKey(keyMin)) {
            prefs.putDouble(keyMin, currentMin);
        }
        if (!prefs.containsKey(keyMax)) {
            prefs.putDouble(keyMax, currentMax);
        }

        currentMin = prefs.getDouble(keyMin, currentMin);
        currentMax = prefs.getDouble(keyMax, currentMax); 
 
        System.out.printf("%s %f %s %f\n", keyMin, currentMin, keyMax, currentMax);
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


    public static void configCurrentLimit(TalonSRX talonSRX)
    {
        talonSRX.configPeakCurrentDuration(50, 10);
        talonSRX.configPeakCurrentLimit(40, 10);
        talonSRX.configContinuousCurrentLimit(35, 10);
        talonSRX.enableCurrentLimit(true);
    }
}
