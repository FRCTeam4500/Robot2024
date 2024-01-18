package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotorController;

public class Arm extends SubsystemBase implements LoggableInputs {

    // Make Motor(s?) here (SparkMaxMotorController)
    private SparkMaxMotorController armMotor;

    // private spark max shooter tilt TODO

    // Make Instance Here (Look At Intake.java) @Author The Yellow
    private static Arm instance;

    // Make GetInstance Here (Look At Intake.java)
    public static synchronized Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }
    // Make Constructor Here
    public Arm()
    {

    }
    @Override
    public void toLog(LogTable table) {

    }

    @Override
    public void fromLog(LogTable table) {}
    public void setState(ArmState state) {
        armMotor.setAngle(new Rotation2d(state.position));//se
    }
}
