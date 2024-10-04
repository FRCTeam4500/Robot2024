package frc.robot.subsystems.intake.real;

import org.littletonrobotics.junction.LogTable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CANConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeThroughBore extends IntakeIO {
    private DutyCycleEncoder throughBore;
    private CANSparkMax tiltMotor;
    private CANSparkMax runMotor;
    private DigitalInput limitSwitch;
    private MechanismLigament2d mech;
    private PIDController pid;
    private boolean tracking;
    private boolean wasTracking;
    private double target;
    public IntakeThroughBore() {
        throughBore = new DutyCycleEncoder(7);
        tiltMotor = new CANSparkMax(CANConstants.INTAKE_TILT_ID, MotorType.kBrushless);
        runMotor = new CANSparkMax(CANConstants.INTAKE_OUTPUT_ID, MotorType.kBrushless);
        limitSwitch = new DigitalInput(CANConstants.INTAKE_ZEROING_LIMIT_SWITCH_ID);
        mech = new MechanismLigament2d("Intake State", 0/4, 0.1);
        MechanismRoot2d root = Superstructure.getCurrentMech().getRoot("Intake Root", 0.3, 45);
        root.append(mech);
        pid = new PIDController(5, 0, 0);
        tracking = false;
        wasTracking = false;
        target = 0;

        throughBore.setPositionOffset(0);
        throughBore.reset();
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Output", runMotor.get());
        table.put("Tilt", throughBore.get());
        table.put("Target", target);
        mech.setAngle(45 - 360 * throughBore.get());
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public void periodic() {
        if (!limitSwitch.get()) {
            throughBore.setPositionOffset(0);
            throughBore.reset();
        }
        if (tracking) {
            tiltMotor.setVoltage(pid.calculate(throughBore.get(), target));
            wasTracking = true;
        } else if (wasTracking) {
            wasTracking = false;
            tiltMotor.setVoltage(0);
        }
    }

    @Override
    public Command tilt(double tilt) {
        return Commands.runOnce(() -> {
            tracking = true;
            target = tilt;
        }, this);
    }

    @Override
    public Command zero() {
        return Commands.runOnce(() -> tracking = false).andThen(Commands.run(
            () -> tiltMotor.set(0.6), this
        )).until(() -> !limitSwitch.get()).withTimeout(2).andThen(() -> tiltMotor.set(0));
    }

    @Override
    public Command run(double output) {
        return Commands.runOnce(
            () -> runMotor.set(output), this
        );
    }

    @Override
    public Command coast() {
        return Commands.runOnce(() -> {
            tracking = false;
        }, this);
    }

    @Override
    public Trigger hasNote() {
        return new Trigger(
            () -> throughBore.get() > 0.5 && runMotor.getEncoder().getVelocity() < 100
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Speed", () -> runMotor.get(), null);
        builder.addDoubleProperty("Velocity", () -> runMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Tilt", () -> throughBore.get(), null);
        builder.addDoubleProperty("Target", () -> target, null);
        builder.addBooleanProperty("Switch", () -> !limitSwitch.get(), null);
    }
    
}
