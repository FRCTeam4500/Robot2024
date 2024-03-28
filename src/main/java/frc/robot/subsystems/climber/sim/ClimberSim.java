package frc.robot.subsystems.climber.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberIO;

public class ClimberSim extends ClimberIO {
    private double currentExtension;
    private Mechanism2d currentMech;
    private MechanismRoot2d root;
    private MechanismLigament2d climberMech;
    public ClimberSim() {
        currentExtension = 0;
        currentMech = Superstructure.getCurrentMech();
        root = currentMech.getRoot("Climber Root", 0.6858, 0.1);
        climberMech = new MechanismLigament2d("Climber State", 0.25, 90);
        root.append(climberMech);
    }
    @Override
    public void toLog(LogTable table) {
        table.put("Extension", currentExtension);
        climberMech.setLength(-currentExtension * 0.004 + 0.25);
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public Command extend(double extension) {
        return Commands.runOnce(() -> {
            currentExtension = extension;
        }, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Extension", () -> currentExtension, null);
    }
    
}
