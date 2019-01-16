/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Add your docs here.
 */
public class TalonPID implements PIDInterface, Sendable {
    private TalonSRX talon;
    private ControlMode controlMode;
    private String name;
    private String subsystem;
    private static final int kTimeoutNs = 10;
    public TalonPID(TalonSRX talon, ControlMode controlMode) {
        this.talon = talon;
        this.controlMode = controlMode;
        this.name = String.format("TalonSRX #%d", talon.getDeviceID());
    }
    @Override
    public void setPID(double p, double i, double d) {
        talon.config_kP(0, p, kTimeoutNs);
        talon.config_kI(0, i, kTimeoutNs);
        talon.config_kD(0, d, kTimeoutNs);
    }
    @Override
    public double getP() {
        return talon.configGetParameter(ParamEnum.eProfileParamSlot_P, 0, kTimeoutNs);
    }
    @Override
    public double getI() {
        return talon.configGetParameter(ParamEnum.eProfileParamSlot_I, 0, kTimeoutNs);
    }
    @Override
    public double getD() {
        return talon.configGetParameter(ParamEnum.eProfileParamSlot_D, 0, kTimeoutNs);
    }
    public double getF() {
        return talon.configGetParameter(ParamEnum.eProfileParamSlot_F, 0, kTimeoutNs);
    }

    public void setP(double p) {
        talon.config_kP(0, p, kTimeoutNs);
    }
    public void setI(double i) {
        talon.config_kI(0, i, kTimeoutNs);
    }
    public void setD(double d) {
        talon.config_kD(0, d, kTimeoutNs);
    }
    public void setF(double f) {
        talon.config_kF(0, f, kTimeoutNs);
    }

    public void enable() {
        talon.set(controlMode, talon.getClosedLoopTarget(0));
    }
    public void disable() {
        talon.set(ControlMode.Velocity, 0.0);
    }
    public boolean isEnabled() {
        return talon.getControlMode() != ControlMode.Velocity;
    }
    public void setEnabled(boolean enabled) {
        if (enabled) {
            enable();
        } else {
            disable();
        }
    }

    @Override
    public void reset() { }

    @Override
    public void setSetpoint(double setPoint) {
        talon.set(controlMode, setPoint);
    }
    @Override
    public double getSetpoint() {
        if (controlMode == ControlMode.Current) {
            return talon.getClosedLoopTarget() / 1000.0;
        } else {
            return talon.getClosedLoopTarget();
        }
    }

    @Override
    public double getError() {
        if (controlMode == ControlMode.Current) {
            return talon.getClosedLoopError() / 1000.0;
        } else {
            return talon.getClosedLoopError();
        }
    }

    @Override
    public String getName() {
        return name;
    }
    @Override
    public void setName(String name) {
        this.name = name;
    }
    
    @Override
    public String getSubsystem() {
        return subsystem;
    }
    @Override
    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.setSafeState(this::reset);
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
    }
}
