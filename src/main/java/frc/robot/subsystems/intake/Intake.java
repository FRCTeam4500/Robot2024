package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.CANConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase implements LoggableInputs {
    private static Intake instance;
    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    private SparkMaxMotorController tiltMotor;
    private SparkMaxMotorController outputMotor;
    private DigitalInput noteLimitSwitch;
    private IntakeState targetState;
    protected Intake() {
        tiltMotor = new SparkMaxMotorController(INTAKE_TILT_ID, MotorType.kBrushless);
        outputMotor = new SparkMaxMotorController(INTAKE_OUTPUT_ID, MotorType.kBrushless);
        noteLimitSwitch = new DigitalInput(INTAKE_NOTE_LIMIT_SWITCH_ID);
        targetState = IntakeState.Stow;
    }

    public void setState(IntakeState state) {
        tiltMotor.setAngle(state.tilt);
        outputMotor.setOutput(state.output);
    }

    public boolean atTargetState() {
        return ExtendedMath.within(tiltMotor.getAngle(), targetState.tilt, tiltThreshold);
    }

    public boolean hasNote() {
        return noteLimitSwitch.get();
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Current Tilt (deg)", tiltMotor.getAngle().getDegrees());
        table.put("Current Output (%)", outputMotor.getOutput());
        table.put("Target Tilt (deg)", targetState.tilt.getDegrees());
        table.put("Target Output (%)", outputMotor.getOutput());
    }

    @Override
    public void fromLog(LogTable table) {}
}