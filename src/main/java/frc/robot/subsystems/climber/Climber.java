package frc.robot.subsystems.climber;

import frc.robot.utilities.ExtendedMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.CANConstants.CLIMBER_ID;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static com.revrobotics.CANSparkLowLevel.MotorType.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

/**
 * @author Gasya
 * @author Ashwin
 * @author Bennett
 */
public class Climber extends SubsystemBase implements LoggableInputs {

    private CANSparkMax motor;
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
        motor = new CANSparkMax(CLIMBER_ID, kBrushless);
        targetState = ClimberState.Lowhook;
    }

    public void setState(ClimberState state) {
        motor.getPIDController().setReference(state.tilt.getRotations(), ControlType.kVelocity);
        targetState = state;
    }

    public boolean atTargetState() {
        return ExtendedMath.within(Rotation2d.fromRotations(motor.getEncoder().getPosition()), targetState.tilt, climberTiltThreshold);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Motor extension (deg)", motor.getEncoder().getPosition());
        table.put("Target State", targetState.name());
    }

    @Override
    public void fromLog(LogTable table) {

    }


}
