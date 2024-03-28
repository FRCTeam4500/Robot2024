package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.real.Shooter;
import frc.robot.subsystems.shooter.sim.ShooterSim;

public abstract class ShooterIO extends SubsystemBase implements LoggableInputs {
    private static ShooterIO instance;
    public static synchronized ShooterIO getInstance() {
        if (instance == null && RobotBase.isReal()) instance = new Shooter();
        if (instance == null && !RobotBase.isReal()) instance = new ShooterSim();
        return instance;
    }
    
    public static final double AMP_TILT = -16.3;
    public static final double HANDOFF_TILT = -7.0;
    public static final double SUBWOOFER_TILT = 1.5;
    public static final double FAR_TILT = -2.2;
    public static final double STOW_TILT = -0.5;

    public static final double SHOOTING_OUTPUT = 1;
    public static final double AMPING_OUTPUT = 0.5;
    public static final double OFF_OUTPUT = 0;

    public static final double LOADER_HANDOFF_OUTPUT = -0.75;
    public static final double LOADER_SHOOTING_OUTPUT = -1;
    public static final double LOADER_OFF_OUTPUT = 0;

    public abstract Command pivot(double tilt);
    public abstract Command autoPivot();
    public abstract Command load(double output);
    public abstract Command spinUp(double left, double right);
    public abstract Command coast();
    public abstract void initSendable(SendableBuilder builder);
}
