package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSpeed;

import static frc.robot.CANConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.outputThreshold;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Represent the intake of our robot
 * @author Vimal Buckley
 */
public class Intake extends SubsystemBase implements LoggableInputs {
    // Singleton stuff
    private static Intake instance;
    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    // Declaring the motors for this subsystem
    private SparkMaxMotorController tiltMotor;
    private SparkMaxMotorController outputMotor;

    // Limit switches
    private DigitalInput pieceLimitSwitch;
    private DigitalInput zeroingLimitSwitch;

    private IntakePosition position;
    private IntakeSpeed speed;

    protected Intake() {
        // Set our motors to values
        tiltMotor = new SparkMaxMotorController(INTAKE_TILT_ID, MotorType.kBrushless);
        outputMotor = new SparkMaxMotorController(INTAKE_OUTPUT_ID, MotorType.kBrushless);

        // Set upon the limit switch
        pieceLimitSwitch = new DigitalInput(INTAKE_NOTE_LIMIT_SWITCH_ID);
        zeroingLimitSwitch = new DigitalInput(INTAKE_ZEROING_LIMIT_SWITCH_ID);

        outputMotor.configBrakeOnIdle(true);

        position = IntakePosition.Zero;
        speed = IntakeSpeed.Off;
    }

    @Override
    public void periodic() {
        if (zeroingLimitSwitch.get()) {
            tiltMotor.assignCurrentAngle(IntakePosition.Zero.tilt);
        }
    }

    /**
     * Set the position and speed of the intake
     * @param targetPosition The position to set the intake to
     * @param targetSpeed The speed to set the intake to
     */
    public void setState(IntakePosition targetPosition, IntakeSpeed targetSpeed) {
        position = targetPosition;
        speed = targetSpeed;
        tiltMotor.setAngle(position.tilt);
        outputMotor.setOutput(speed.output);
    }

    /**
     * Set the position of the intake
     * @param targetPosition The position to set the intake to
     */
    public void setState(IntakePosition targetPosition) {
        setState(targetPosition, speed);
    }

    /**
     * Set the speed of the intake
     * @param targetSpeed The speed to set the intake to
     */
    public void setState(IntakeSpeed targetSpeed) {
        setState(position, targetSpeed);
    }

    /**
     * Is the intake at the target position?
     * @return Whether or not the intake is at the target position
     */
    public boolean atPosition() {
        return Math.abs(tiltMotor.getAngle().minus(position.tilt).getDegrees()) 
            < positionThreshold.getDegrees();
    }

    /**
     * Is the intake at the target speed?
     * @return Whether or not the intake is at the target speed
     */
    public boolean atSpeed() {
        return Math.abs(outputMotor.getOutput() - speed.output) < outputThreshold;
    }

    public boolean hasNote() {
        return pieceLimitSwitch.get();
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Intake Angle", tiltMotor.getAngle().getDegrees());
        table.put("Intake Speed", outputMotor.getOutput());
        table.put("Target Angle", position.tilt.getDegrees());
        table.put("Target Speed", speed.output);
    }

    @Override
    public void fromLog(LogTable table) {}
}