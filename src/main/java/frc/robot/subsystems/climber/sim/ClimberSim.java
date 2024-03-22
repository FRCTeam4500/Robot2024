package frc.robot.subsystems.climber.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberIO;

public class ClimberSim extends ClimberIO {
    private double currentExtension;
    public ClimberSim() {
        currentExtension = 0;
    }
    @Override
    public void toLog(LogTable table) {
        table.put("Extension", currentExtension);
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public Command extend(double extension) {
        return Commands.runOnce(() -> currentExtension = extension, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Extension", () -> currentExtension, null);
    }
    
}
