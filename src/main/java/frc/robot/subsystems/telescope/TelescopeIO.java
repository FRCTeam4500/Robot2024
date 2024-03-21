package frc.robot.subsystems.telescope;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.telescope.real.Telescope;
import frc.robot.subsystems.telescope.sim.TelescopeSim;

public abstract class TelescopeIO extends SubsystemBase implements LoggableInputs {
    public static TelescopeIO instance;
    public static synchronized TelescopeIO getInstance() {
        if (instance == null && RobotBase.isReal()) instance = new Telescope();
        if (instance == null && !RobotBase.isReal()) instance = new TelescopeSim();
        return instance;
    }
    public static final double AMP_EXTENSION = 3000;
    public static final double SHOOTING_EXTENSION = 3500;
    public static final double AUTO_EXTENSION = 3000;
    public static final double HANDOFF_EXTENSION = 400;
    public abstract Command extend(double extension);
    public abstract Command coast();
    public abstract void initSendable(SendableBuilder builder);
}
