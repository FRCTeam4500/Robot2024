package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.CANConstants.*;

public class Intake extends SubsystemBase {
    private static Intake instance;
    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }
    
    public static final double GROUND_TILT = -62;
    public static final double STOW_TILT = -18;
    public static final double HANDOFF_TILT = -3;
    public static final double PICKUP_SPEED = 0.3;
    public static final double OFF_SPEED = 0;
    public static final double HANDOFF_SPEED = -0.17;

    private CANSparkMax tiltMotor;
    private CANSparkMax runMotor;
    private DigitalInput limitSwitch;
    public Intake() {
        runMotor = new CANSparkMax(INTAKE_OUTPUT_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(INTAKE_TILT_ID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(INTAKE_ZEROING_LIMIT_SWITCH_ID);

        tiltMotor.getPIDController().setOutputRange(-0.6, 0.6);
        tiltMotor.getPIDController().setP(1);
    }

    @Override
    public void periodic() {
        if (!limitSwitch.get()) {
            tiltMotor.getEncoder().setPosition(0);
        }
    }

    public Command tilt(double angle) {
        return Commands.runOnce(
            () -> tiltMotor.getPIDController().setReference(angle, ControlType.kPosition),
            this
        );
    }

    public Command run(double output) {
        return Commands.runOnce(
            () -> runMotor.set(output)
        );
    }

    public Command reset() {
        return Commands.runOnce(() -> tiltMotor.getEncoder().setPosition(0));
    }

    public Command coast() {
        return Commands.runOnce(() -> tiltMotor.set(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Speed", () -> runMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Tilt", () -> tiltMotor.getEncoder().getPosition(), null);
    }
}
