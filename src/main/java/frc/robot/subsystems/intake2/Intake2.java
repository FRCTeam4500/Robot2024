package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake2 extends SubsystemBase {

    private static Intake2 instance;
    public static synchronized Intake2 getInstance() {
        if (instance == null) instance = new Intake2();
        return instance;
    }

    private CANSparkMax tiltMotor;
    private CANSparkMax runMotor;
    public Intake2() {
        runMotor = new CANSparkMax(10, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(11, MotorType.kBrushless);

        tiltMotor.getPIDController().setOutputRange(-0.4, 0.4);
        tiltMotor.getPIDController().setP(1);
    }

    public Command setTilt(double angle) {
        return Commands.runOnce(
            () -> tiltMotor.getPIDController().setReference(angle, ControlType.kPosition),
            this
        );
    }

    public Command setOutput(double output) {
        return Commands.runOnce(
            () -> runMotor.set(output)
        );
    }

    public Command zeroTilt() {
        return Commands.runOnce(() -> tiltMotor.getEncoder().setPosition(0));
    }

    public Command coastTilt() {
        return Commands.runOnce(() -> tiltMotor.set(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Speed", () -> runMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Tilt", () -> tiltMotor.getEncoder().getPosition(), null);
    }
}
