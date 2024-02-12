package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.swerve.SwerveConstants.DriveMode;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Swerve extends SubsystemBase implements LoggableInputs {

	private static Swerve instance;
	public static synchronized Swerve getInstance() {
		if (instance == null) instance = new Swerve();
		return instance;
	}

	private NavX gyro;
	private AprilTagVision tagVision;
	private GamePieceVision pieceVision;
	private SwerveModule[] modules;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveDrivePoseEstimator poseEstimator;
	private PIDController anglePID;
    private Rotation2d targetAngle;
    private DriveMode driveMode;

	protected Swerve() {
		anglePID = new PIDController(7, 0, 0);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
		modules = new SwerveModule[] {
			new SwerveModule(
				FRONT_LEFT_DRIVE_MOTOR,
				FRONT_LEFT_ANGLE_MOTOR,
				FRONT_LEFT_MODULE_TRANSLATION
			),
			new SwerveModule(
				FRONT_RIGHT_DRIVE_MOTOR,
				FRONT_RIGHT_ANGLE_MOTOR,
				FRONT_RIGHT_MODULE_TRANSLATION
			),
			new SwerveModule(
				BACK_LEFT_DRIVE_MOTOR,
				BACK_LEFT_ANGLE_MOTOR,
				BACK_LEFT_MODULE_TRANSLATION
			),
			new SwerveModule(
				BACK_RIGHT_DRIVE_MOTOR,
				BACK_RIGHT_ANGLE_MOTOR,
				BACK_RIGHT_MODULE_TRANSLATION
			),
		};
		gyro = new NavX(edu.wpi.first.wpilibj.SPI.Port.kMXP);
		tagVision = AprilTagVision.getInstance();
		pieceVision = GamePieceVision.getInstance();
		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		odometry = new SwerveDriveOdometry(
			kinematics,
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			// tagVision.getRobotPose(new Pose2d())
			new Pose2d(8, 4, getRobotAngle())
		);
		poseEstimator = new SwerveDrivePoseEstimator(
			kinematics,
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			tagVision.getRobotPose(new Pose2d())
		);

		targetAngle = getRobotAngle();
        driveMode = DriveMode.AngleCentric;
	}

	public boolean gyroConnected() {
		return gyro.getAHRS().isConnected();
	}

	@Override
	public void periodic() {
		Rotation2d gyroAngle = gyro.getUnwrappedAngle();
		SwerveModulePosition[] modulePositions = getModulePositions();
		odometry.update(gyroAngle, modulePositions);
		poseEstimator.update(gyroAngle, modulePositions);
		if (tagVision.seesTag()) {
			if (DriverStation.isAutonomous()) {
				poseEstimator.addVisionMeasurement(
					tagVision.getRobotPose(getEstimatorPose()),
					Timer.getFPGATimestamp()
            	);
			} else {
				poseEstimator.addVisionMeasurement(
					new Pose2d(tagVision.getRobotPose(getEstimatorPose()).getTranslation(), gyro.getOffsetedAngle()),
					Timer.getFPGATimestamp()
				);
			}

		}
	}

	public void driveAngleCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		Rotation2d targetRotation
	) {
		driveRobotCentric(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				calculateRotationalVelocityToTarget(targetRotation),
				getRobotAngle()
			)
		);
	}

	public void driveAlignToTarget(
		double forwardVelocity,
		double leftVelocity,
		Rotation2d aligningAngle
	) {
		driveRobotCentric(
			new ChassisSpeeds(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					forwardVelocity,
					leftVelocity,
					0,
					getRobotAngle()
				).vxMetersPerSecond,
				pieceVision.getHorizontalOffset(new Rotation2d()).getDegrees() / (pieceVision.getTakenArea(99)),
				calculateRotationalVelocityToTarget(aligningAngle)
			)
		);
	}

	public void driveRobotCentric(ChassisSpeeds targetChassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			ChassisSpeeds.discretize(targetChassisSpeeds, 0.02)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			MAX_LINEAR_SPEED_MPS
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	public void resetPose(Pose2d newPose) {
		odometry.resetPosition(
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			newPose
		);
		poseEstimator.resetPosition(
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			newPose
		);
		resetRobotAngle(newPose.getRotation().plus(Rotation2d.fromDegrees(180)));
	}

	public void zeroRobotAngle() {
		gyro.zero();
	}

	public void resetRobotAngle(Rotation2d offsetAngle) {
		gyro.zeroWithOffset(offsetAngle);
	}

	/* COMMANDS */

	public Command angleCentricDrive(CommandXboxController xbox) {
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
            }, this
        );
    }

    /**
     * turns robot to speaker
     *
     * @param xbox
     *
     * @author Bennett
     * @author David
     */
    public Command rotateToSpeaker(CommandXboxController xbox)
    {
        return Commands.run(() -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;

                driveAngleCentric(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    ExtendedMath.getAngleToSpeaker(getEstimatorPose())
                );
            }, this
        );
    }

    public Command resetGyro() {
        return Commands.runOnce(
            () -> {
                zeroRobotAngle();
                targetAngle = new Rotation2d();
            }
        );
    }

	/* TELEMETRY */

	@Override
	public void toLog(LogTable table) {
		table.put(
			"Front Left Module Velocity (M/S)",
			modules[0].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Front Left Module Angle (Radians)",
			modules[0].getModuleState().angle.getRadians()
		);
		table.put(
			"Front Right Module Velocity (M/S)",
			modules[1].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Front Right Module Angle (Radians)",
			modules[1].getModuleState().angle.getRadians()
		);
		table.put(
			"Back Left Module Velocity (M/S)",
			modules[2].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Back Left Module Angle (Radians)",
			modules[2].getModuleState().angle.getRadians()
		);
		table.put(
			"Back Right Module Velocity (M/S)",
			modules[3].getModuleState().speedMetersPerSecond
		);
		table.put(
			"Back Right Module Angle (Radians)",
			modules[3].getModuleState().angle.getRadians()
		);
		Logger.recordOutput(
			"Swerve Odometry",
			getOdometryPose()
		);
		Logger.recordOutput(
			"Odometyry + Vision Pose Estimation",
			getEstimatorPose()
		);
		Logger.recordOutput(
			"Module States",
			getModuleStates()
		);
	}

	@Override
	public void fromLog(LogTable table) {}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Offset Robot Angle (deg)", () -> gyro.getOffsetedAngle().getDegrees(), null);
		builder.addDoubleProperty("Forward Velocity (mps)", () -> getChassisSpeeds().vxMetersPerSecond, null);
		builder.addDoubleProperty("Sideways Velocity (mps)", () -> getChassisSpeeds().vyMetersPerSecond, null);
		builder.addDoubleProperty("Rotational Velocity (radps)", () -> getChassisSpeeds().omegaRadiansPerSecond, null);
		builder.addBooleanProperty("Gyro Connected", () -> gyro.getAHRS().isConnected(), null);
		builder.addStringProperty("Drive Mode", () -> driveMode.name(), null);
        builder.addDoubleProperty("Target Angle (Deg)", () -> targetAngle.getDegrees(), null);
	}

	/* HELPERS AND GETTERS */

	private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		double rotationalVelocity = anglePID.calculate(
			getRobotAngle().getRadians(),
			targetRotation.getRadians()
		);
		if (anglePID.atSetpoint()) {
			rotationalVelocity = 0;
		}
		return rotationalVelocity;
	}

	public Rotation2d getRobotAngle() {
		return gyro.getOffsetedAngle();
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	public Pose2d getEstimatorPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Pose2d getOdometryPose() {
		return odometry.getPoseMeters();
	}

	private Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getModuleState();
		}
		return states;
	}

	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getModulePosition();
		}
		return positions;
	}
}