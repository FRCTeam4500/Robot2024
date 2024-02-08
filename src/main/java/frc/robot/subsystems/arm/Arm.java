package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.utilities.ExtendedMath;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.hardware.TalonSRXMotor;

/**
 * @author Max
 * @author Yijia
 * @author Oliver (not) The Short
 */
public class Arm extends SubsystemBase implements LoggableInputs {

    // Make Motor(s?) here (SparkMaxMotorController)
    private TalonSRXMotor extensionMotor; 

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
        extensionMotor = new TalonSRXMotor(CANConstants.ARM_EXTENSION_MOTOR_ID);
        extensionMotor.config_kP(0, 1);
        targetState = ArmState.ZERO;
    }

    public boolean atTargetState() { // returns your target state
        return ExtendedMath.within(extensionMotor.getSelectedSensorPosition(), targetState.extension, ArmConstants.extensionThreshold);
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
        extensionMotor.set(ControlMode.MotionMagic, state.extension);
    }
}
