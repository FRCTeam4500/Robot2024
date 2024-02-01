package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.utilities.ExtendedMath;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;
import frc.robot.hardware.SparkMaxMotor;

/**
 * @author Max
 * @author Yijia
 * @author Oliver The Smart
 */
public class Arm extends SubsystemBase implements LoggableInputs {

    // Make Motor(s?) here (SparkMaxMotorController)
    private SparkMaxMotor extensionMotor; 

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
        extensionMotor = new SparkMaxMotor(CANConstants.ARM_EXTENSION_MOTOR_ID);
        targetState = ArmState.ZERO;
    }

    public boolean atTargetState() { //returns your target state
        return ExtendedMath.within(extensionMotor.getAngle(), targetState.extension, ArmConstants.extensionThreshold);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Current Extension (deg)", extensionMotor.getAngle().getDegrees());
        table.put("Target State", targetState.name());
    }

    @Override
    public void fromLog(LogTable table) {
    }

    public void setState(ArmState state) {
        extensionMotor.setAngle(state.extension);
    }
}
