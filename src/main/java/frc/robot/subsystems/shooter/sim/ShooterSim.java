package frc.robot.subsystems.shooter.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swerve.SwerveIO;

public class ShooterSim extends ShooterIO {
    private double currentTilt;
    private double currentloaderOutput;
    private double currentRightShooterOutput;
    private double currentLeftShooterOutput;
    private InterpolatingDoubleTreeMap angleCalculator;
    public ShooterSim() {
        currentTilt = 0;
        currentloaderOutput = 0;
        currentRightShooterOutput = 0;
        currentLeftShooterOutput = 0;
        
        angleCalculator = new InterpolatingDoubleTreeMap();
        angleCalculator.put(1.37, 1.25);
        angleCalculator.put(1.74, -0.25);
        angleCalculator.put(2.14, -2.25);
        angleCalculator.put(2.8, -3.25);
        angleCalculator.put(3.25, -3.75);
        angleCalculator.put(3.75, -4.25);
        angleCalculator.put(4.4, -4.6);
    }

    public Command pivot(double tilt) {
        return Commands.runOnce(() -> currentTilt = tilt, this); 
    }

    public Command autoPivot() {
        return Commands.run(() -> {
            Translation2d current = SwerveIO.getInstance().getEstimatedPose().getTranslation();
            Translation2d speaker = new Translation2d(16, 5.6);
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                speaker = new Translation2d(0, 5.6);
            }
            currentTilt = angleCalculator.get(current.getDistance(speaker));
        }, this);
    }

    public Command load(double output) {
        return Commands.runOnce(() -> currentloaderOutput = output, this);
    }

    public Command spinUp(double left, double right) {
        return Commands.runOnce(() -> {
            currentLeftShooterOutput = left;
            currentRightShooterOutput = right;
        }, this);
    }

    public Command coast() {
        return Commands.runOnce(() -> currentTilt = 0, this);
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Left Output", () -> currentLeftShooterOutput, null);
        builder.addDoubleProperty("Right Output", () -> currentRightShooterOutput, null);
        builder.addDoubleProperty("Loader Output", () -> currentloaderOutput, null);
        builder.addDoubleProperty("Tilt", () -> currentTilt, null);
    }

    public void toLog(LogTable table) {
        table.put("Left Output", currentLeftShooterOutput);
        table.put("Right Output", currentRightShooterOutput);
        table.put("Loader Output", currentloaderOutput);
        table.put("Tilt", currentTilt);
    } 

    public void fromLog(LogTable table) {}
}
