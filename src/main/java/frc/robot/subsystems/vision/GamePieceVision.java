package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;

public class GamePieceVision extends SubsystemBase implements LoggableInputs {
    private static GamePieceVision instance;
    public static synchronized GamePieceVision getInstance() {
        if (instance == null) instance = new GamePieceVision();
        return instance;
    }

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
}
