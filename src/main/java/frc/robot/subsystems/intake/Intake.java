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

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Intake extends SubsystemBase implements LoggableInputs {
    private static Intake instance;
    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    public static final double GROUND_TILT = -55;
    public static final double STOW_TILT = 0;
    public static final double HANDOFF_TILT = 0;
    public static final double AMP_TILT = -10;
    public static final double PICKUP_SPEED = 0.5;
    public static final double OFF_SPEED = 0;
    public static final double HANDOFF_SPEED = -1;
    public static final double SHOOTING_SPEED = -1;
    public static final double AMP_SPEED = -1;

    private CANSparkMax tiltMotor;
    private CANSparkMax runMotor;
    private DigitalInput limitSwitch;
    public Intake() {
        runMotor = new CANSparkMax(INTAKE_OUTPUT_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(INTAKE_TILT_ID, MotorType.kBrushless);

        // Wires are backwards, black is signal, white is ground
        limitSwitch = new DigitalInput(INTAKE_ZEROING_LIMIT_SWITCH_ID);

        tiltMotor.getPIDController().setOutputRange(-0.75, 0.75);
        tiltMotor.getPIDController().setP(5);

        tiltMotor.getEncoder().setPosition(0);

        tiltMotor.setSmartCurrentLimit(30);
        runMotor.setSmartCurrentLimit(30);
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

    public Command zeroIntake() {
        return Commands.run(
            () -> tiltMotor.set(0.6)
        ).until(() -> !limitSwitch.get()).withTimeout(2).andThen(() -> tiltMotor.set(0));
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
        builder.addDoubleProperty("Speed", () -> runMotor.get(), null);
        builder.addDoubleProperty("Tilt", () -> tiltMotor.getEncoder().getPosition(), null);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Output", runMotor.get());
        table.put("Tilt", tiltMotor.getEncoder().getPosition());
    }

    @Override
    public void fromLog(LogTable table) {}
}
