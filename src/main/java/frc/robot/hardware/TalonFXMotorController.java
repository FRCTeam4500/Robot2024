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

public class TalonFXMotorController extends TalonFX implements EncodedMotorController {
    private TalonFXConfiguration config;
    private MotionMagicVoltage mVoltage;
    private VelocityVoltage vVoltage;

    public TalonFXMotorController(int deviceID) {
        super(deviceID);
        config = new TalonFXConfiguration();
        mVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
        vVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(false);
    }

    public TalonFXMotorController(
        int deviceID,
        TalonFXConfiguration config
    ) {
        super(deviceID);
        this.config = config;
        mVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
        vVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(false);
        refreshConfig();
    }

    @Override
    public void setAngularVelocity(Rotation2d velocity) {
        setControl(vVoltage.withVelocity(velocity.getRotations()));
    }

    @Override
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRotations(getVelocity().getValueAsDouble());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        setControl(mVoltage.withPosition(angle.getRotations()));
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(getPosition().getValueAsDouble());
    }

    @Override
    public void setOutput(double output) {
        set(output);
    }

    @Override
    public double getOutput() {
        return get();
    }

    @Override
    public boolean hasContinuousRotation() {
        return false;
    }

    @Override
    public TalonFXMotorController configCurrentLimit(int limtAmps) {
        config.CurrentLimits.SupplyCurrentLimit = limtAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentThreshold = limtAmps + 1;
        refreshConfig();
        return this;
    }

    @Override
    public TalonFXMotorController configAnglePID(PIDConstants pid) {
        config.Slot0.kP = pid.kP;
        config.Slot0.kI = pid.kI;
        config.Slot0.kD = pid.kD;
        refreshConfig();
        return this;
    }

    @Override
    public TalonFXMotorController configVelocityPID(PIDConstants pid) {
        config.Slot1.kP = pid.kP;
        config.Slot1.kI = pid.kI;
        config.Slot1.kD = pid.kD;
        refreshConfig();
        return this;
    }

    @Override
    public TalonFXMotorController configMinAngle(Rotation2d min) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min.getRotations();
        refreshConfig();
        return this;    
    }

    @Override
    public TalonFXMotorController configMaxAngle(Rotation2d max) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max.getRotations();
        refreshConfig();
        return this;
    }

    @Override
    public TalonFXMotorController configMinOutput(double min) {
        config.MotorOutput.PeakReverseDutyCycle = ExtendedMath.clamp(-1, 1, min);
        refreshConfig();
        return this;
    }

    @Override
    public TalonFXMotorController configMaxOutput(double max) {
        config.MotorOutput.PeakForwardDutyCycle = ExtendedMath.clamp(-1, 1, max);
        refreshConfig();
        return this;
    }

    @Override
    public TalonFXMotorController configInverted(boolean shouldInvert) {
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

    @Override
    public TalonFXMotorController configBrakeOnIdle(boolean shouldBreak) {
        if (shouldBreak) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        refreshConfig();
        return this;
    }
    
    public TalonFXMotorController configMotionMagic(double maxVelocity, double acceleration) {
        config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        refreshConfig();
        return this;
    }

    public TalonFXMotorController configAngleFF(double kV) {
        config.Slot0.kV = kV;
        refreshConfig();
        return this;
    }

    public TalonFXMotorController configVelocityFF(double kV) {
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
