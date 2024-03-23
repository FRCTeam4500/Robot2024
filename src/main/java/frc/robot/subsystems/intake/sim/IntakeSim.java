package frc.robot.subsystems.intake.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeSim extends IntakeIO {
    private double currentTilt;
    private double currentOutput;
    private Mechanism2d currentMech;
    private MechanismRoot2d root;
    private MechanismLigament2d intakeMech;
    public IntakeSim() {
        currentTilt = 0;
        currentOutput = 0;
        currentMech = Superstructure.getCurrentMech();
        root = currentMech.getRoot("Intake Root", 0.4, 0.1);
        intakeMech = new MechanismLigament2d("Intake State", 0.5, 45);
        root.append(intakeMech);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Tilt", currentTilt);
        table.put("Output", currentOutput);
        intakeMech.setAngle(-2.70909 * currentTilt + 45);
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public Command tilt(double tilt) {
        return Commands.runOnce(() -> currentTilt = tilt, this);
    }

    @Override
    public Command zero() {
        return Commands.runOnce(() -> currentTilt = 0, this);
    }

    @Override
    public Command run(double output) {
        return Commands.runOnce(() -> currentOutput = output, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Tilt", () -> currentTilt, null);
        builder.addDoubleProperty("Output", () -> currentOutput, null);
    }
    
}
