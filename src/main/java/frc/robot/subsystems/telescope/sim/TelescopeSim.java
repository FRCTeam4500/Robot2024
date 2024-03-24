package frc.robot.subsystems.telescope.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.telescope.TelescopeIO;

public class TelescopeSim extends TelescopeIO {
    private double currentExtension;
    private Mechanism2d currentMech;
    private MechanismRoot2d root;
    private MechanismLigament2d telescopeState;
    public TelescopeSim() {
        currentExtension = 0;
        currentMech = Superstructure.getCurrentMech();
        root = currentMech.getRoot("Telescope Root", 0.889, 0.1524);
        telescopeState = new MechanismLigament2d("Telescope State", 0.4064, 75);
        root.append(telescopeState);
    }
    @Override
    public void toLog(LogTable table) {
        table.put("Extension", currentExtension);
        telescopeState.setLength(0.000166667 * currentExtension + 0.4);
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public Command extend(double extension) {
        return Commands.runOnce(() -> currentExtension = extension, this);
    }

    @Override
    public Command coast() {
        return Commands.runOnce(() -> currentExtension = 0, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Extension", () -> currentExtension, null);
    }

    @Override
    public MechanismLigament2d getCurrentMech() {
        return telescopeState;
    }
    
}
