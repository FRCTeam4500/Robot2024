package frc.robot.subsystems.pieceVision.real;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.pieceVision.GamePieceVisionIO;

public class GamePieceVision extends GamePieceVisionIO {
    private final double LIMELIGHT_HEIGHT_METERS = 0.232;
	private final double GAMEPIECE_HALF_HEIGHT_METERS = 0.16;
	private final Rotation2d LIMELIGHT_ANGLE = Rotation2d.fromDegrees(-12);
    private Limelight limelight;

    public GamePieceVision() {
        limelight = new Limelight("limelight-haha");
    }

    public boolean seesPiece() {
        return limelight.hasValidTargets();
    }

    public Translation2d getTranslation(Translation2d defaultTranslation) {
        if (!seesPiece()) return defaultTranslation;
		double forwardDistance = 
			(LIMELIGHT_HEIGHT_METERS - GAMEPIECE_HALF_HEIGHT_METERS) / 
			Math.tan(
				LIMELIGHT_ANGLE.plus(
					getVerticalOffset(new Rotation2d())
				).getRadians()
			);
		return new Translation2d(
			forwardDistance,
			forwardDistance * Math.tan(
				getVerticalOffset(new Rotation2d())
				.getRadians()
			)
		);
    }

    public Rotation2d getHorizontalOffset(Rotation2d defaultOffset) {
        return limelight.getHorizontalOffsetFromCrosshair().orElse(defaultOffset);
    }

    public Rotation2d getVerticalOffset(Rotation2d defaultOffset) {
        return limelight.getVerticalOffsetFromCrosshair().orElse(defaultOffset);
    }

    public double getTakenArea(double defaultArea) {
        return limelight.getTargetArea().orElse(defaultArea);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Sees Piece", seesPiece());
        Logger.recordOutput(
            "Piece Translation", 
            new Pose2d(getTranslation(new Translation2d()), new Rotation2d())
        );
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Horizontal Offset", () -> getHorizontalOffset(new Rotation2d()).getDegrees(), null);
        builder.addDoubleProperty("Vertical Offset", () -> getVerticalOffset(new Rotation2d()).getDegrees(), null);
        builder.addDoubleProperty("Taken Area", () -> getTakenArea(0), null);
        builder.addBooleanProperty("Sees Piece", () -> seesPiece(), null);
    }
}
