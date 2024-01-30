package frc.robot.subsystems.climber;

import frc.robot.CANConstants;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.utilities.ExtendedMath;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * @author Gasya
 * @author Ashwin
 * @author Bennett
 */
public class Climber extends SubsystemBase implements LoggableInputs {

    private SparkMaxMotor motor;
    private ClimberState targetState;

    private static Climber instance;
    public static synchronized Climber getInstance() {
        if (instance == null) instance = new Climber ();
        return instance;

    }

    /**
     * @author Gasya
     * @author Ashwin
     * @author Bennett
     */
    public Climber() {
        motor = new SparkMaxMotor(CANConstants.CLIMBER_ID);
        targetState = ClimberState.Lowhook;
    }

    public void setState(ClimberState state) {
        motor.setAngle(state.tilt);
        targetState = state;
    }

    public boolean atTargetState() {
        return ExtendedMath.within(motor.getAngle(), targetState.tilt, climberTiltThreshold);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Motor extension (deg)", motor.getAngle().getDegrees());
        table.put("Target State", targetState.name());
    }
    @Override
    public void fromLog(LogTable table) {

    }
    

}
