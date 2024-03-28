package frc.robot.subsystems.pieceVision;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GamePieceVisionIO extends SubsystemBase implements LoggableInputs {
    private static GamePieceVisionIO instance;
    public static synchronized GamePieceVisionIO getInstance() {
        return instance;
    }

    public abstract boolean seesPiece();
    public abstract Rotation2d getHorizontalOffset(Rotation2d defaultRotation);
    public abstract Rotation2d getVerticalOffset(Rotation2d defaultRotation);
    public abstract double getTakenArea(double defaultArea);
    public abstract void initSendable(SendableBuilder builder);
}
