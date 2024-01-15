package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveConstants.DriveMode;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class CommandSwerve extends SwerveDrive {
    private static CommandSwerve instance;
    public static synchronized CommandSwerve getInstance() {
        if (instance == null) instance = new CommandSwerve();
        return instance;
    }

    private AprilTagVision tagVision;
    private GamePieceVision pieceVision;
    private Rotation2d targetAngle;
    private DriveMode driveMode;
    private CommandSwerve() {
        super();
        tagVision = AprilTagVision.getInstance();
        pieceVision = GamePieceVision.getInstance();
        targetAngle = getRobotAngle();
        driveMode = DriveMode.AngleCentric;
    }

    public Command angleCentricDriveCommand(CommandXboxController xbox) {
        return Commands.run(
            () -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;
                double rotationalSens = MAX_ROTATIONAL_SENSITIVITY * coefficent;
                if (Math.abs(xbox.getRightY()) > 0.5){
                    targetAngle = Rotation2d.fromDegrees(90 - 90 * Math.signum(-xbox.getRightY()));
                }
                targetAngle = Rotation2d.fromDegrees(
                    targetAngle.getDegrees() - 
                    xbox.getRightX() * rotationalSens
                );
                driveAngleCentric(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    targetAngle
                );
            }, this
        ).beforeStarting(() -> {
            targetAngle = getRobotAngle();
            driveMode = DriveMode.AngleCentric;
        });
    }

    public Command robotCentricDrive(CommandXboxController xbox) {
        return Commands.run(() -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;
                double rotationalSens = MAX_ROTATIONAL_SENSITIVITY * coefficent;

                driveRobotCentric(new ChassisSpeeds(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    -xbox.getRightX() * rotationalSens
                ));
            }, this
        ).beforeStarting(() -> driveMode = DriveMode.RobotCentric);
    }

    public Command alignToPiece(CommandXboxController xbox) {
        return Commands.run(() -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;

                driveAlignToTarget(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    targetAngle
                );
            }, this
        ).beforeStarting(() -> {
            targetAngle = getRobotAngle();
            driveMode = DriveMode.AlignToTarget;
        });
    }

    public Command driveToTag(Pose2d targetPose) {
        return Commands.run(
			() -> {
				Pose2d poseDif = tagVision.getRelativeTagPose(targetPose).relativeTo(targetPose);
				driveRobotCentric(
                    new ChassisSpeeds(
						poseDif.getX() * 2,
						poseDif.getY() * 2,
						-poseDif.getRotation().getRadians() * 5
					)
				);
			}, this
		);
    }

    public Command driveToPiece() {
        return Commands.run(
            () -> {
                if (!pieceVision.seesPiece()) {
                    driveRobotCentric(new ChassisSpeeds());
                } else {
                    Rotation2d diff = pieceVision.getHorizontalOffset(new Rotation2d());
                    double area = pieceVision.getTakenArea(40);
                    driveRobotCentric(new ChassisSpeeds(
                        ExtendedMath.clamp(0, 1, 20 - area),
                        5 * diff.getRadians(), 
                        0
                    ));
                }
            }, this);
    }

    public Command resetGyro() {
        return Commands.runOnce(
            () -> {
                zeroRobotAngle();
                targetAngle = new Rotation2d();
            }
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Drive Mode", () -> driveMode.name(), null);
        builder.addDoubleProperty("Target Angle (Deg)", () -> targetAngle.getDegrees(), null);
    }
}
