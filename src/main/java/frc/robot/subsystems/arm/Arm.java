package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.utilities.ExtendedMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;
import frc.robot.hardware.SparkMaxMotorController;

/**
 * @author Max
 * @author Yijia
 * @author Oliver
 */
public class Arm extends SubsystemBase implements LoggableInputs {

    // Make Motor(s?) here (SparkMaxMotorController)
    private SparkMaxMotorController extensionMotor;
    private SparkMaxMotorController tiltMotor;

    // private spark max shooter tilt TODO

    // Make Instance Here (Look At Intake.java) @author The Yellow
    private static Arm instance;

    // Make GetInstance Here (Look At Intake.java)
    public static synchronized Arm getInstance() {
        if (instance == null)
            instance = new Arm();
        return instance;
    }

    private ArmState targetState;

    // Make Constructor Here
    public Arm() {
        extensionMotor = new SparkMaxMotorController(CANConstants.ARM_EXTENSION_MOTOR_ID, MotorType.kBrushless);
        tiltMotor = new SparkMaxMotorController(CANConstants.ARM_TILT_MOTOR_ID, MotorType.kBrushless);
        targetState = ArmState.ZERO;
    }

    public boolean atTargetState() { //returns your target state
        return ExtendedMath.within(extensionMotor.getAngle().getDegrees(), targetState.extensionPosition, ArmConstants.extensionPositionThreshold) 
        && ExtendedMath.within(tiltMotor.getAngle().getDegrees(), targetState.tiltPosition, ArmConstants.tiltPositionThreshold);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Current Tilt (deg)", tiltMotor.getAngle().getDegrees());
        table.put("Current Extension (deg)", extensionMotor.getAngle().getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
    }

    public void setState(ArmState state) {
        extensionMotor.setAngle(Rotation2d.fromDegrees(state.extensionPosition));// se
        tiltMotor.setAngle(Rotation2d.fromDegrees(state.tiltPosition));
    }
}
