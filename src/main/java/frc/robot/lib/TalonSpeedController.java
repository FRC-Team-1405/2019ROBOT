/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

public class TalonSpeedController implements SpeedController {

    private WPI_TalonSRX talonSRX ;
    private ControlMode controlMode = ControlMode.PercentOutput;

    private static double INPUT_MIN = -1.0;
    private static double INPUT_MAX =  1.0;
    private double outputMin = -1.0;
    private double outputMax =  1.0;
    private double slope = (outputMax - outputMin) / (INPUT_MAX - INPUT_MIN);

    public TalonSpeedController(WPI_TalonSRX talonSRX){
        this.talonSRX = talonSRX;
    }

    public TalonSpeedController setMode(ControlMode mode, double min, double max) {
        controlMode = mode;
        outputMin = min;
        outputMax = max;
        slope = (outputMax - outputMin) / (INPUT_MAX - INPUT_MIN);
        return this;
    }

    public double scaleOutput(double output){
        return outputMin + slope * (output - INPUT_MIN);
    }

    @Override
    public void pidWrite(double output) {
        talonSRX.set(controlMode, scaleOutput(output)) ;
    }

    @Override
    public void set(double speed) {
        talonSRX.set(controlMode, scaleOutput(speed)) ;
    }

    @Override
    public double get() {
        return talonSRX.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        talonSRX.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return talonSRX.getInverted();
    }

    @Override
    public void disable() {
        talonSRX.disable();
    }

    @Override
    public void stopMotor() {
        talonSRX.stopMotor();
    }
}
