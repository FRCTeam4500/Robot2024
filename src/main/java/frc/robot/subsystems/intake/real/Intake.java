package frc.robot.subsystems.intake.real;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeIO;

import static frc.robot.CANConstants.*;

import org.littletonrobotics.junction.LogTable;

public class Intake extends IntakeIO {private CANSparkMax tiltMotor;
    private CANSparkMax runMotor;
    private DigitalInput limitSwitch;
    private Mechanism2d currentMech;
    private MechanismRoot2d root;
    private MechanismLigament2d intakeMech;
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

        currentMech = Superstructure.getCurrentMech();
        root = currentMech.getRoot("Intake Root", 0.4, 0.1);
        intakeMech = new MechanismLigament2d("Intake State", 0.3, 45);
        root.append(intakeMech);
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

    public Command zero() {
        return Commands.run(
            () -> tiltMotor.set(0.6), this
        ).until(() -> !limitSwitch.get()).withTimeout(2).andThen(() -> tiltMotor.set(0));
    }

    public Command run(double output) {
        return Commands.runOnce(
            () -> runMotor.set(output), this
        );
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
        intakeMech.setAngle(-2.70909 * tiltMotor.getEncoder().getPosition() + 45);
    }

    @Override
    public void fromLog(LogTable table) {}
}
