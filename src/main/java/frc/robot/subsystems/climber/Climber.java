package frc.robot.subsystems.climber;

import frc.robot.hardware.SparkMaxMotorController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * @author Gasya
 * @author Ashwin
 */
public class Climber extends SubsystemBase implements LoggableInputs {

    private SparkMaxMotorController leftMotor;
    private SparkMaxMotorController rightMotor;
    private ClimberState targetState;

    private static Climber instance;
    public static synchronized Climber getInstance(){
        if (instance == null) instance = new Climber ();
        return instance;

    }

    /**
     * @author Gasya
     * @author Ashwin
     */
    public Climber() {
        leftMotor = new SparkMaxMotorController(ClimberConstants.LEFTMOTORID, MotorType.kBrushless);
        rightMotor = new SparkMaxMotorController(ClimberConstants.RIGHTMOTORID, MotorType.kBrushless);
        targetState=ClimberState.Lowhook;
    }

    public void setState(ClimberState state){
        leftMotor.setAngle(state.tilt);
        rightMotor.setAngle(state.tilt);
        targetState = state;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Target Tilt (deg)", targetState.tilt.getDegrees());
        table.put("Right extension (deg)", rightMotor.getAngle().getDegrees());
        table.put("Left extension (deg)", leftMotor.getAngle().getDegrees());
    }
    @Override
    public void fromLog(LogTable table) {

    }


}
