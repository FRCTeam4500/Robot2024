package frc.robot.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utilities.ExtendedMath;
import frc.robot.subsystems.swerve.SwerveMotor;

public class TalonFXMotor extends TalonFX implements SwerveMotor {
    private TalonFXConfiguration config;
    private MotionMagicVoltage mVoltage;
    private VelocityVoltage vVoltage;

    public TalonFXMotor(int deviceID) {
        this(deviceID, new TalonFXConfiguration());
    }

    public TalonFXMotor(int deviceID, TalonFXConfiguration config) {
        super(deviceID);
        this.config = config;
        setPosition(0);
        mVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
        vVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(false);
        refreshConfig();
    }

    public void setAngularVelocity(Rotation2d velocity) {
        setControl(vVoltage.withVelocity(velocity.getRotations()));
    }

    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRotations(super.getVelocity().getValueAsDouble());
    }

    public void setAngle(Rotation2d angle) {
        setControl(mVoltage.withPosition(angle.getRotations()));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(getPosition().getValueAsDouble());
    }

    public void setOutput(double output) {
        set(output);
    }

    public double getOutput() {
        return get();
    }

    public boolean hasContinuousRotation() {
        return false;
    }

    public TalonFXMotor configCurrentLimit(int limtAmps) {
        config.CurrentLimits.SupplyCurrentLimit = limtAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentThreshold = limtAmps + 1;
        refreshConfig();
        return this;
    }

    public TalonFXMotor configAnglePID(PIDConstants pid) {
        config.Slot0.kP = pid.kP;
        config.Slot0.kI = pid.kI;
        config.Slot0.kD = pid.kD;
        refreshConfig();
        return this;
    }

    public TalonFXMotor configVelocityPID(PIDConstants pid) {
        config.Slot1.kP = pid.kP;
        config.Slot1.kI = pid.kI;
        config.Slot1.kD = pid.kD;
        refreshConfig();
        return this;
    }

    public TalonFXMotor configMinAngle(Rotation2d min) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min.getRotations();
        refreshConfig();
        return this;
    }

    public TalonFXMotor configMaxAngle(Rotation2d max) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max.getRotations();
        refreshConfig();
        return this;
    }

    public TalonFXMotor configMinOutput(double min) {
        config.MotorOutput.PeakReverseDutyCycle = ExtendedMath.clamp(-1, 1, min);
        refreshConfig();
        return this;
    }

    public TalonFXMotor configMaxOutput(double max) {
        config.MotorOutput.PeakForwardDutyCycle = ExtendedMath.clamp(-1, 1, max);
        refreshConfig();
        return this;
    }

    public TalonFXMotor configInverted(boolean shouldInvert) {
        if (shouldInvert) {
            if (config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) {
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            } else {
                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }
        }
        refreshConfig();
        return this;
    }

    public TalonFXMotor configBrakeOnIdle(boolean shouldBreak) {
        if (shouldBreak) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        refreshConfig();
        return this;
    }

    public TalonFXMotor configMotionMagic(double maxVelocity, double acceleration) {
        config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        refreshConfig();
        return this;
    }

    public TalonFXMotor configAngleFF(double kV) {
        config.Slot0.kV = kV;
        refreshConfig();
        return this;
    }

    public TalonFXMotor configVelocityFF(double kV) {
        config.Slot1.kV = kV;
        refreshConfig();
        return this;
    }

    private boolean refreshConfig() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        return status.isOK();
    }
}
