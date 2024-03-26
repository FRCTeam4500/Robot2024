package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.real.Intake;
import frc.robot.subsystems.intake.sim.IntakeSim;

public abstract class IntakeIO extends SubsystemBase implements LoggableInputs {
    private static IntakeIO instance;
    public static synchronized IntakeIO getInstance() {
        if (instance == null && RobotBase.isReal()) instance = new Intake();
        if (instance == null && !RobotBase.isReal()) instance = new IntakeSim();
        return instance;
    }

    public static final double GROUND_TILT = -37.5;
    public static final double EJECT_SPEED = -1;
    public static final double PICKUP_SPEED = 0.5;
    public static final double OFF_SPEED = 0.0;

    public abstract Command tilt(double tilt);
    public abstract Command zero();
    public abstract Command run(double output);
    public abstract void initSendable(SendableBuilder builder);

}
