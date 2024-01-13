package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class TalonSRXMotorController extends TalonSRX implements EncodedMotorController {
    private double TICKS_PER_RADIAN = 4096 / Math.PI / 2;

    public TalonSRXMotorController(int deviceID) {
        super(deviceID);
    }

    @Override
    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
    }

    @Override
    public double getOutput() {
        return getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(Rotation2d targetAngularVelocity) {
        set(ControlMode.Velocity, targetAngularVelocity.getRadians() * TICKS_PER_RADIAN / 10.0);
    }

    @Override
    public Rotation2d getAngularVelocity() {
        return new Rotation2d(getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10);
    }

    @Override
    public void setAngle(Rotation2d targetAngle) {
        set(ControlMode.Position, targetAngle.getRadians() * TICKS_PER_RADIAN);
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(getSelectedSensorPosition() / TICKS_PER_RADIAN);
    }

    @Override
    public boolean hasContinuousRotation() {
        return true;
    }

    @Override
    public TalonSRXMotorController configCurrentLimit(int currentLimit) {
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

    @Override
    public TalonSRXMotorController configAnglePID(PIDConstants pid) {
        config_kP(0, pid.kP);
        config_kI(0, pid.kI);
        config_kD(0, pid.kD);
        return this;
    }

    @Override
    public EncodedMotorController configVelocityPID(PIDConstants pid) {
        return configAnglePID(pid);
    }

    @Override
    public TalonSRXMotorController configMinAngle(Rotation2d minPosition) {
        configReverseSoftLimitEnable(true);
        configReverseSoftLimitThreshold(minPosition.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public TalonSRXMotorController configMaxAngle(Rotation2d maxPosition) {
        configForwardSoftLimitEnable(true);
        configForwardSoftLimitThreshold(maxPosition.getRadians() * TICKS_PER_RADIAN);
        return this;
    }

    @Override
    public TalonSRXMotorController configMinOutput(double minOutput) {
        configPeakOutputReverse(minOutput);
       return this;
    }

    @Override
    public TalonSRXMotorController configMaxOutput(double maxOutput) {
        configPeakOutputForward(maxOutput);
        return this;
    }
    
    @Override
    public TalonSRXMotorController configInverted(boolean shouldInvert) {
        setInverted(shouldInvert);
        return this;
    }

    @Override
    public TalonSRXMotorController configBrakeOnIdle(boolean shouldBreak) {
        setNeutralMode(
            shouldBreak
            ? NeutralMode.Brake
            : NeutralMode.Coast
        );
        return this;
    }
}