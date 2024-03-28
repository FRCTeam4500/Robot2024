package frc.robot.subsystems.pieceVision.sim;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.pieceVision.GamePieceVisionIO;

public class GamePieceVisionSim extends GamePieceVisionIO {
    @Override
    public void toLog(LogTable table) {
        table.put("Sees Piece", seesPiece());
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public boolean seesPiece() {
        return false;
    }

    @Override
    public Rotation2d getHorizontalOffset(Rotation2d defaultRotation) {
        return defaultRotation;
    }

    @Override
    public Rotation2d getVerticalOffset(Rotation2d defaultRotation) {
        return defaultRotation;
    }

    @Override
    public double getTakenArea(double defaultArea) {
        return defaultArea;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Horizontal Offset", () -> getHorizontalOffset(new Rotation2d()).getDegrees(), null);
        builder.addDoubleProperty("Vertical Offset", () -> getVerticalOffset(new Rotation2d()).getDegrees(), null);
        builder.addDoubleProperty("Taken Area", () -> getTakenArea(0), null);
        builder.addBooleanProperty("Sees Piece", () -> seesPiece(), null);
    }

    
}
