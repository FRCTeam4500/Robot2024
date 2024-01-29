package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.utilities.ExtendedMath;
import static frc.robot.CANConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

/**
 * This class represents the robot's i1ntake. It has a lot of comments, use it
 * as a referense for other subsystems. You should probably be using
 * {@link CommandIntake CommandIntake} instead for
 * actual roboting.
 * @author Vimal Buckley
 */
public class Intake extends SubsystemBase implements LoggableInputs {
    // Static: Maintains one state throughout all intake objects
    // Not Static: Has a state for each intake object

    // Singleton Stuff
    private static Intake instance;
    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    // Declare variables
    // Hardware
    private SparkMaxMotor tiltMotor;
    private SparkMaxMotor outputMotor;
    // private DigitalInput noteLimitSwitch;
    // private DigitalInput zeroingLimitSwitch;
    // State
    private IntakeState targetState;

    /**
     * The constructor of the intake class. This is run when an object of type Intake
     * is created. Initialization code (setting variables to values) should
     * happen here.
     * @author Vimal Buckley
     */
    protected Intake() {
        tiltMotor = new SparkMaxMotor(INTAKE_TILT_ID, MotorType.kBrushless);
        outputMotor = new SparkMaxMotor(INTAKE_OUTPUT_ID, MotorType.kBrushless);
        // noteLimitSwitch = new DigitalInput(INTAKE_NOTE_LIMIT_SWITCH_ID);
        // zeroingLimitSwitch = new DigitalInput(INTAKE_ZEROING_LIMIT_SWITCH_ID);
        targetState = IntakeState.Stow;
    }

    public void setTilt(Rotation2d target) {
        tiltMotor.setAngle(target);
    }

    public void setSpeed(double output) {
        outputMotor.set(output);
    }

    /**
     * Sets the state of the intake
     * Doesn't return anything (return type is void)
     * @param state The new target state of the intake
     * @author Vimal Buckley
     */
    public void setState(IntakeState state) {
        tiltMotor.setAngle(state.tilt);
        outputMotor.setOutput(state.output);
        targetState = state;
    }

    /**
     * Returns a boolean (return type is boolean)
     * @return Whether the intake has reached its target tilt
     * @author Vimal Buckley & Billy Bonga
     */
    public boolean atTargetState() {
        return ExtendedMath.within(tiltMotor.getAngle(), targetState.tilt, tiltThreshold);
    }

    /**
     * Returns a boolean (return type is boolean)
     * @return Whether the limit switch is detecting a note in the intake
     * @author Vimal Buckley
     */
    public boolean hasNote() {
        // return noteLimitSwitch.get();
        return false;
    }

    /**
     * Method that runs every 20ms
     * The Override annotation shows that it is from the
     * {@link edu.wpi.first.wpilibj2.command.SubsystemBase SubsystemBase} class
     * @author Vimal Buckley
     */
    @Override
    public void periodic() {
        // If the limit switch is triggered (if we're stowed), reset the encoder
        // of the arm motor. This is so that the arm motor will be constantly
        // corrected throughout the match
        // if (zeroingLimitSwitch.get()) {
        //     tiltMotor.getEncoder()
        //         .setPosition(IntakeState.Stow.tilt.getRotations());
        // }
    }

    /**
     * Method that logs data to a data table.
     * The Override annoation shows that it is from the
     * {@link org.littletonrobotics.junction.inputs.LoggableInputs LoggedInputs}
     * interface
     * @param table The table to log data to
     * @author Vimal Buckley
     */
    @Override
    public void toLog(LogTable table) {
        table.put("Current Tilt (deg)", tiltMotor.getAngle().getDegrees());
        table.put("Current Output (%)", outputMotor.getOutput());
        table.put("Target Tilt (deg)", targetState.tilt.getDegrees());
        table.put("Target Output (%)", outputMotor.getOutput());
    }

    /**
     * A method to update a robot's state from a log. Useful for simulation,
     * which we don't do
     * @param table The table to read state from
     * @author Vimal Buckley
     */
    @Override
    public void fromLog(LogTable table) {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Tilt (deg)", () -> tiltMotor.getAngle().getDegrees(), null);
    }
}