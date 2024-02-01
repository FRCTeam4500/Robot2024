package frc.robot.subsystems.shooter;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static frc.robot.CANConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.utilities.ExtendedMath;

/**
 * @author David Wharton
 * @author Gretchen Miller
 */
public class Shooter extends SubsystemBase implements LoggableInputs {

    private static Shooter instance;


    public static synchronized Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    /**
     * word we use for the first motor
     */
    private SparkMaxMotor shootshootMotorOne;


    // word we use for the second motor

    private SparkMaxMotor shootshootMotorTwo;

    /**
     * word we use for the loading motor
     */
    private SparkMaxMotor loaderMotor;

    /**
     * Motor to tilt the shooter
     */
    private SparkMaxMotor tiltMotor;
    /**
 * wanted state of the shooter/loader
 */

    private ShooterState targetState;
    /**
 * defines motors
 * @author David Wharton
 * @author lord gre
 */

    protected Shooter() {
        shootshootMotorOne = new SparkMaxMotor(SHOOTER_ONE_ID);
        shootshootMotorTwo = new SparkMaxMotor(SHOOTER_TWO_ID);
        loaderMotor = new SparkMaxMotor(LOADER_ID);
        tiltMotor = new SparkMaxMotor(SHOOTER_PIVOT_ID);
        targetState = ShooterState.Off;
    }
  /**
 * changes target state
 * @author David Wharton
 * @author Gretchen Miller
 */
    public void setTargetState(ShooterState state) {
        shootshootMotorOne.setOutput(state.shooterSpeed);
        shootshootMotorTwo.setOutput(state.shooterSpeed);
        loaderMotor.setOutput(state.loaderSpeed);
        tiltMotor.setAngle(state.tilt);
        targetState = state;
    }
/**
 * checks if the motor is spun up
 * @author David Wharton
 * @author Gretchen Miller
 */
    public boolean spunUp() {
        return Math.abs(shootshootMotorOne.getOutput() - targetState.shooterSpeed) < speedThreshold &&
                Math.abs(shootshootMotorTwo.getOutput() - targetState.shooterSpeed) < speedThreshold;
    }

    public boolean tilted() {
        return ExtendedMath.within(tiltMotor.getAngle(), targetState.tilt, tiltThreshold);
    }

    /**
     * log stuff in the graphy tihng
     * 
     * @author Lord Gretchyn (the david copy and pasted the loader motor part but i
     *         made the original)
     * @param David
     */
    @Override
    public void toLog(LogTable table) {
        table.put("Current shootshoot motor output 1 (%):", shootshootMotorOne.getOutput());
        table.put("Current shootshoot motor output 2 (%):", shootshootMotorTwo.getOutput());
        table.put("Current loader motor output (%):", loaderMotor.getOutput());
        table.put("Shooter tilt (deg)", tiltMotor.getAngle().getDegrees());
        table.put("Current state", targetState.name());
    }
/**
 * does absolutely nothing
 * @author David Wharton
 * @author Gretchen Miller
 */
    @Override
    public void fromLog(LogTable table) {}
}
