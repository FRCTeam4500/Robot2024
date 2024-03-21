package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public interface SwerveMotor {
    public void setAngle(Rotation2d target);
    public void setAngularVelocity(Rotation2d target);
    public Rotation2d getAngle();
    public Rotation2d getAngularVelocity();

    public static SwerveMotor fromTalonSRX(TalonSRX motor, Consumer<TalonSRX> config) {
        config.accept(motor);
        return new SwerveMotor() {
            public void setAngle(Rotation2d target) {
                motor.set(ControlMode.Position, target.getRotations() * 4096);
            }
            public void setAngularVelocity(Rotation2d target) {
                motor.set(ControlMode.Velocity, target.getRotations() * 4096 / 10);
            }
            public Rotation2d getAngle() {
                return Rotation2d.fromRotations(motor.getSelectedSensorPosition() / 4096);
            }
            public Rotation2d getAngularVelocity() {
                return Rotation2d.fromRotations(motor.getSelectedSensorPosition() / 4096 * 10);
            }
        };
    }

    public static SwerveMotor fromTalonFX(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        motor.setPosition(0);
        return new SwerveMotor() {
            MotionMagicVoltage mVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
            VelocityVoltage vVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(false);
            public void setAngle(Rotation2d target) {
                motor.setControl(mVoltage.withPosition(target.getRotations()));
            }
            public void setAngularVelocity(Rotation2d target) {
                motor.setControl(vVoltage.withVelocity(target.getRotations()));
            }
            public Rotation2d getAngle() {
                return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
            }
            public Rotation2d getAngularVelocity() {
                return Rotation2d.fromRotations(motor.getVelocity().getValueAsDouble());
            }
        };
    }

    public static SwerveMotor fromSparkMax(CANSparkMax motor, Consumer<CANSparkMax> config) {
        config.accept(motor);
        return new SwerveMotor() {
            public void setAngle(Rotation2d target) {
                motor.getPIDController().setReference(target.getRotations(), ControlType.kPosition);
            }
            public void setAngularVelocity(Rotation2d target) {
                motor.getPIDController().setReference(target.getRotations() * 60, ControlType.kVelocity);
            }
            public Rotation2d getAngle() {
                return Rotation2d.fromRotations(motor.getEncoder().getPosition());
            }
            public Rotation2d getAngularVelocity() {
                return Rotation2d.fromRotations(motor.getEncoder().getVelocity() / 60);
            }
        };
    }
}

