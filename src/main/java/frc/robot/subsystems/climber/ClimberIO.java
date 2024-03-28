package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.real.Climber;
import frc.robot.subsystems.climber.sim.ClimberSim;

public abstract class ClimberIO extends SubsystemBase implements LoggableInputs {
    private static ClimberIO instance;
    public static synchronized ClimberIO getInstance() {
        if (instance == null && RobotBase.isReal()) instance = new Climber();
        if (instance == null && !RobotBase.isReal()) instance = new ClimberSim();
        return instance;
    }
    public static final double ZERO = 0.0;
    public static final double EXTENDED = -130;
    public static final double RETRACTED = 30;
    public abstract Command extend(double extension);
    public abstract void initSendable(SendableBuilder builder);
}
