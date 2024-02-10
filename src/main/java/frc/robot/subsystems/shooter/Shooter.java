package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.CANConstants.*;

public class Shooter extends SubsystemBase {

    private static Shooter instance;
    public static synchronized Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    public static final double AMP_TILT = -18;
    public static final double HANDOFF_TILT = -6.15;
    public static final double SUBWOOFER_TILT = 1;
    public static final double STOW_TILT = 0;
    public static final double SUBWOOFER_LEFT_SPEED = -0.9;
    public static final double SUBWOOFER_RIGHT_SPEED = -0.7;
    public static final double AMP_SPEED = -0.5;
    public static final double OFF_SPEED = 0;
    public static final double LOADER_HANDOFF_SPEED = -0.18;
    public static final double LOADER_SHOOT_SPEED = -1;
    public static final double LOADER_OFF_SPEED = 0;

    private CANSparkMax tiltMotor;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax loaderMotor;
    private AnalogInput lidar;
    private Shooter() {
        rightMotor = new CANSparkMax(SHOOTER_ONE_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(SHOOTER_TWO_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(SHOOTER_PIVOT_ID, MotorType.kBrushless);
        loaderMotor = new CANSparkMax(LOADER_ID, MotorType.kBrushless);
        lidar = new AnalogInput(LOADER_LIDAR_CHANNEL);

        rightMotor.getPIDController().setP(0.3);
        leftMotor.getPIDController().setP(0.3);
        tiltMotor.getPIDController().setP(0.5);
        tiltMotor.getPIDController().setOutputRange(-0.3, 0.3);
    }

    public boolean hasNote() {
        return lidar.getValue() < 2200;
    }

    public Command pivot(double angle) {
        return Commands.runOnce(
            () -> tiltMotor.getPIDController().setReference(angle, ControlType.kPosition),
            this
        );
    }

    public Command load(double output) {
        return Commands.runOnce(
            () -> loaderMotor.set(output), this
        );
    }

    public Command waitTillNote() {
        return Commands.waitUntil(() -> hasNote());
    }

    public Command shoot(double left, double right) {
        return Commands.runOnce(
            () -> {
                leftMotor.set(left);
                rightMotor.set(right);
            }
        );
    }

    public Command off() {
        return Commands.runOnce(
            () -> {
                leftMotor.set(0);
                rightMotor.set(0);
            }  
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Left Speed", () -> leftMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Right Speed", () -> rightMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Loader Speed", () -> loaderMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Tilt", () -> tiltMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("LIDAR Measurement", () -> lidar.getValue(), null);
    }
}
