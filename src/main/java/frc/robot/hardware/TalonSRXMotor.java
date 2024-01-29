package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveMotor;

public class TalonSRXMotor extends TalonSRX implements SwerveMotor {
    private double TICKS_PER_RADIAN = 4096 / Math.PI / 2;

    public TalonSRXMotor(int deviceID) {
        super(deviceID);
    }

    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
    }

    public double getOutput() {
        return getMotorOutputPercent();
    }

    public void setAngularVelocity(Rotation2d targetAngularVelocity) {
        set(ControlMode.Velocity, targetAngularVelocity.getRadians() * TICKS_PER_RADIAN / 10.0);
    }

    public Rotation2d getAngularVelocity() {
        return new Rotation2d(getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10);
    }

    public void setAngle(Rotation2d targetAngle) {
        set(ControlMode.Position, targetAngle.getRadians() * TICKS_PER_RADIAN);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(getSelectedSensorPosition() / TICKS_PER_RADIAN);
    }

    public boolean hasContinuousRotation() {
        return true;
    }

    public TalonSRXMotor configCurrentLimit(int currentLimit) {
        configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                true, 
                currentLimit, 
                currentLimit + 1, 
                0.1
            ), 
            50
        );
        return this;
    }

    public TalonSRXMotor configAnglePID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    public TalonSRXMotor configVelocityPID(PIDConstants pid) {
        return configAnglePID(pid);
    }

    public TalonSRXMotor configMinAngle(Rotation2d minPosition) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(minPosition.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    public TalonSRXMotor configMaxAngle(Rotation2d maxPosition) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(maxPosition.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    public TalonSRXMotor configMinOutput(double minOutput) {
        configPeakOutputReverse(minOutput);
       return this;
    }

    public TalonSRXMotor configMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }
    
    public TalonSRXMotor configInverted(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    public TalonSRXMotor configBrakeOnIdle(boolean shouldBreak) {
        setNeutralMode(
            shouldBreak
            ? NeutralMode.Brake
            : NeutralMode.Coast
        );
        return this;
    }
}