package frc.robot.subsystems.tagVision.real;


import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.tagVision.AprilTagVisionIO;
import frc.robot.utilities.ExtendedMath;

public class AprilTagVision extends AprilTagVisionIO {
    private Limelight front;
    private Limelight back;

    public AprilTagVision() {
        front = new Limelight("limelight-hehehe");
        back = new Limelight("limelight-haha");
    }

    public boolean seesTag(Camera camera) {
        return getCamera(camera).hasValidTargets();
    }

    public int getTagId(int defaultId, Camera camera) {
        return getCamera(camera).getTargetTagId().orElse(defaultId);
    }

    public Pose2d getRobotPose(Pose2d defaultPose, Camera camera) {
		return getRobotPose(defaultPose, Alliance.Blue, camera);
	}

	public Pose2d getRobotPose(Pose2d defaultPose, Alliance poseOrigin, Camera camera) {
		return getCamera(camera)
			.getRobotPoseToAlliance(poseOrigin)
			.orElse(defaultPose);
	}

    public Pose2d getRelativeTagPose(Pose2d defaultPose, Camera camera) {
        if (!seesTag(camera)) return defaultPose;
		Pose2d backwardsPose = getRobotPose(new Pose2d(), Alliance.Blue, camera)
			.relativeTo(getTagPose(getTagId(0, camera)));
        return new Pose2d(
            backwardsPose.getTranslation(), 
            ExtendedMath.wrapRotation2d(backwardsPose.getRotation()
                .plus(Rotation2d.fromDegrees(180)))
        );
    }
    
    private Pose2d getTagPose(int tagId) {
        // Positions gotten from https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
        // but with flipped rotations, and in meters
        switch (tagId) {
            default:
                return new Pose2d();
            case 1:
                return new Pose2d(15.079472, 0.245872, Rotation2d.fromDegrees(300));
            case 2:
                return new Pose2d(16.185134, 0.883666, Rotation2d.fromDegrees(300));
            case 3:
                return new Pose2d(16.679342, 4.982718, Rotation2d.fromDegrees(0));
            case 4:
                return new Pose2d(16.679342, 5.547868, Rotation2d.fromDegrees(0));
            case 5:
                return new Pose2d(14.700758, 8.2042, Rotation2d.fromDegrees(90));
            case 6:
                return new Pose2d(1.8415, 8.2042, Rotation2d.fromDegrees(90));
            case 7:
                return new Pose2d(-0.0381, 5.547868, Rotation2d.fromDegrees(180));
            case 8:
                return new Pose2d(-0.0381, 4.982718, Rotation2d.fromDegrees(180));
            case 9:
                return new Pose2d(0.356108, 0.883666, Rotation2d.fromDegrees(240));
            case 10:
                return new Pose2d(1.461516, 0.245872, Rotation2d.fromDegrees(240));
            case 11:
                return new Pose2d(11.904726, 3.713226, Rotation2d.fromDegrees(120)); 
            case 12:
                return new Pose2d(11.904726, 4.49834, Rotation2d.fromDegrees(240));
            case 13:
                return new Pose2d(11.220196, 4.105148, Rotation2d.fromDegrees(0));
            case 14:
                return new Pose2d(5.320792, 4.105148, Rotation2d.fromDegrees(180));
            case 15:
                return new Pose2d(4.641342, 4.49834, Rotation2d.fromDegrees(300));
            case 16:
                return new Pose2d(4.641342, 3.713226, Rotation2d.fromDegrees(60)); 
        }
    }

    private Limelight getCamera(Camera camera) {
        switch (camera) {
            case Front:
                return front;
            case Back:
                return back;
        }
        return front;
    }
    
    @Override
    public void toLog(LogTable table) {
        table.put("Front Sees Tag", seesTag(Camera.Front));
        table.put("Front Tag ID", getTagId(0, Camera.Front));
        Logger.recordOutput("Front Vision Robot Pose", getRobotPose(new Pose2d(), Camera.Front));
        table.put("Back Sees Tag", seesTag(Camera.Back));
        table.put("Back Tag ID", getTagId(0, Camera.Back));
        Logger.recordOutput("Back Vision Robot Pose", getRobotPose(new Pose2d(), Camera.Back));
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Front Sees Tag", () -> seesTag(Camera.Front), null);
        builder.addIntegerProperty("Front Tag ID", () -> getTagId(0, Camera.Front), null);
        builder.addBooleanProperty("Back Sees Tag", () -> seesTag(Camera.Back), null);
        builder.addIntegerProperty("Back Tag ID", () -> getTagId(0, Camera.Back), null);
    }
}
