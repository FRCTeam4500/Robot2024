package frc.robot.subsystems.intake.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeSim extends IntakeIO {
    private double currentTilt;
    private double currentOutput;
    public IntakeSim() {
        currentTilt = 0;
        currentOutput = 0;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Tilt", currentTilt);
        table.put("Output", currentOutput);
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public Command tilt(double tilt) {
        return Commands.runOnce(() -> currentTilt = tilt);
    }

    @Override
    public Command zero() {
        return Commands.runOnce(() -> currentTilt = 0);
    }

    @Override
    public Command run(double output) {
        return Commands.runOnce(() -> currentOutput = output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Tilt", () -> currentTilt, null);
        builder.addDoubleProperty("Output", () -> currentOutput, null);
    }
    
}
