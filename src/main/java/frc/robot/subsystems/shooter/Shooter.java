package frc.robot.subsystems.shooter;
//impot s the shooterconatsnts
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
//imports the CANConstants that are needed in the Shooter.java file
import static frc.robot.CANConstants.LOADER_ID;
import static frc.robot.CANConstants.SHOOTER_ONE_ID;
import static frc.robot.CANConstants.SHOOTER_TWO_ID;
///imports the MOTORTypE
import com.revrobotics.CANSparkLowLevel.MotorType;
//imports the SunsystemBase and sparkyadayada
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotorController;
//imports the ShooterConstants and ShooterState
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

/**
 * a bunch of stuff
 * 
 * @author David Wharton
 * @author lord gre
 */
public class Shooter extends SubsystemBase implements LoggableInputs {

    private static Shooter instance;

    // hi
    public static synchronized Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    /**
     * word we use for the first motor
     * 
     * @author David Wharton
     * @author lord gre
     */
    private SparkMaxMotorController shootshootMotorOne;

    /**
     * word we use for the second motor
     * 
     * @author David Wharton
     * @author lord gre
     */
    private SparkMaxMotorController shootshootMotorTwo;

    /**
     * word we use for the loading motor
     * 
     * @author David Wharton
     * @author lord gre
     */
    private SparkMaxMotorController loaderMotor;
    /**
 * wanted state of the shooter/loader
 * @author David Wharton
 * @author lord gre
 */

    private ShooterState targetState;
    /**
 * defines motors
 * @author David Wharton
 * @author lord gre
 */

    protected Shooter() {
        shootshootMotorOne = new SparkMaxMotorController(SHOOTER_ONE_ID, MotorType.kBrushless);
        shootshootMotorTwo = new SparkMaxMotorController(SHOOTER_TWO_ID, MotorType.kBrushless);
        loaderMotor = new SparkMaxMotorController(LOADER_ID, MotorType.kBrushless);
        targetState = ShooterState.Off;
    }
  /**
 * changes target state
 * @author David Wharton
 * @author lord gre
 */
    public void setTargetState(ShooterState state) {
        shootshootMotorOne.setOutput(state.shooterSpeed);
        shootshootMotorTwo.setOutput(state.shooterSpeed);//beans
        loaderMotor.setOutput(state.loaderSpeed);
        targetState = state;
    }
/**
 * checks if the motor is spun up
 * @author David Wharton
 * @author lord gre
 */
    public boolean spunUp() {
        return Math.abs(shootshootMotorOne.getOutput() - targetState.shooterSpeed) < threshold &&
                Math.abs(shootshootMotorTwo.getOutput() - targetState.shooterSpeed) < threshold;
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
        table.put("Current state",targetState.name());
    }
/**
 * does absolutely nothing
 * @author David Wharton
 * @author lord gre
 */
    @Override
    public void fromLog(LogTable table) {
        // i came out to my mom and she got mad at me - The Lord
    }
}
