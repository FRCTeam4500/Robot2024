package frc.robot.subsystems.swerve.real;

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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.hardware.NavX;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveConstants.DriveMode;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

public class Swerve extends SwerveIO {

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

	public Swerve() {
		anglePID = new PIDController(5, 0, 0);
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
			tagVision.getRobotPose(new Pose2d())
		);
		poseEstimator = new SwerveDrivePoseEstimator(
			kinematics,
			gyro.getUnwrappedAngle(),
			getModulePositions(),
			tagVision.getRobotPose(new Pose2d())
		);
		targetAngle = getRobotAngle();
        driveMode = DriveMode.AngleCentric;
		Shuffleboard.getTab("Debug").add("Swerve Drive",new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle", () -> getModuleStates()[0].angle.getDegrees(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);
				builder.addDoubleProperty("Front Right Angle", () -> getModuleStates()[1].angle.getDegrees(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> getModuleStates()[1].speedMetersPerSecond, null);
				builder.addDoubleProperty("Back Left Angle", () -> getModuleStates()[2].angle.getDegrees(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> getModuleStates()[2].speedMetersPerSecond, null);
				builder.addDoubleProperty("Back Right Angle", () -> getModuleStates()[3].angle.getDegrees(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> getModuleStates()[3].speedMetersPerSecond, null);
				builder.addDoubleProperty("Robot Angle", () -> getRobotAngle().getDegrees(), null);
			}
		});
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
		double dist = tagVision.getRelativeTagPose(new Pose2d(100, 100, gyroAngle)).getTranslation().getNorm();
		if (tagVision.seesTag() &&
			(ExtendedMath.within(getChassisSpeeds(), new ChassisSpeeds(), new ChassisSpeeds(0.5, 0.5, 0.5))
			|| !DriverStation.isAutonomous()) && dist < 4
		) {	
			poseEstimator.addVisionMeasurement(tagVision.getRobotPose(new Pose2d()), Timer.getFPGATimestamp());
		}
	}

	public void driveAngleCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		Rotation2d targetRotation
	) {
		Rotation2d addition = new Rotation2d();
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			addition = Rotation2d.fromDegrees(180);
		}
		driveRobotCentric(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				calculateRotationalVelocityToTarget(targetRotation),
				getRobotAngle().plus(addition)
			)
		);
	}

	public void driveFieldCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		Rotation2d addition = new Rotation2d();
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			addition = Rotation2d.fromDegrees(180);
		}
		driveRobotCentric(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity,
				getRobotAngle().plus(addition)
			)
		);
	}

	public void driveAlignToTarget(
		double forwardVelocity,
		double leftVelocity,
		Rotation2d aligningAngle
	) {
		Rotation2d addition = new Rotation2d();
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			addition = Rotation2d.fromDegrees(180);
		}
		driveRobotCentric(
			new ChassisSpeeds(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					forwardVelocity,
					leftVelocity,
					0,
					getRobotAngle().plus(addition)
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
		resetRobotAngle(newPose.getRotation());
	}

	public void zeroRobotAngle() {
		gyro.zero();
	}

	public void resetRobotAngle(Rotation2d offsetAngle) {
		gyro.zeroWithOffset(offsetAngle);
	}

	/* COMMANDS */

	public Command poseCentricDrive(Pose2d target) {
		return AutoBuilder.pathfindToPoseFlipped(target, TELEOP_CONSTRAINTS);
	}

	public Command fieldCentricDrive(CommandXboxController xbox) {
		return Commands.run(
			() -> {
				double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;
                double rotationalSens = MAX_ROTATIONAL_SENSITIVITY * coefficent;
				driveFieldCentric(
					-xbox.getLeftY() * forwardSens,
					-xbox.getLeftX() * sidewaysSens,
					-xbox.getRightX() * rotationalSens
				);
			}
		).beforeStarting(() -> {
			driveMode = DriveMode.FieldCentric;
		});
	}

	public Command angleCentricDrive(CommandXboxController xbox) {
		return Commands.run(
            () -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;
				double rotationalSens = MAX_ROTATIONAL_SENSITIVITY * coefficent;
				double angleCoefficient = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;
                if (Math.abs(xbox.getRightY()) > 0.5)
                    targetAngle = Rotation2d.fromDegrees(90 + angleCoefficient * 90 * Math.signum(-xbox.getRightY()));
				else if (xbox.getHID().getRightStickButton())
					targetAngle = Rotation2d.fromDegrees(-90);
				else if (xbox.getHID().getLeftBumper())
					targetAngle = Rotation2d.fromDegrees(90);
				else 
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
            driveMode = DriveMode.XanderDrive;
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

    public Command driveToPiece() {
        return Commands.run(
            () -> {
                if (!pieceVision.seesPiece()) {
                    driveRobotCentric(new ChassisSpeeds());
                } else {
                    Rotation2d diff = pieceVision.getHorizontalOffset(new Rotation2d());
                    double area = pieceVision.getTakenArea(40);
                    driveRobotCentric(new ChassisSpeeds(
                        ExtendedMath.clamp(0, 1, 30 - area),
                        3 * diff.getRadians(),
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
    public Command speakerCentricDrive(CommandXboxController xbox) {
        return Commands.run(() -> {
                double coefficent = Math.max(1 - xbox.getLeftTriggerAxis(), 0.2);
                double forwardSens = MAX_FORWARD_SENSITIVITY * coefficent;
                double sidewaysSens = MAX_SIDEWAYS_SENSITIVITY * coefficent;

                driveAngleCentric(
                    -xbox.getLeftY() * forwardSens,
                    -xbox.getLeftX() * sidewaysSens,
                    ExtendedMath.getSpeakerAngle(getEstimatedPose().getTranslation())
                );
            }, this
        );
    }

    public Command resetGyro() {
        return Commands.runOnce(
            () -> {
				if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
					targetAngle = Rotation2d.fromDegrees(180);
				} else {
					targetAngle = new Rotation2d();
				}
                resetRobotAngle(targetAngle);
            }
        );
    }

	/* TELEMETRY */

	@Override
	public void toLog(LogTable table) {
		Logger.recordOutput(
			"Odometric Pose",
			getOdometryPose()
		);
		Logger.recordOutput(
			"Estimated Pose",
			getEstimatedPose()
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
        builder.addDoubleProperty("Target Angle (deg)", () -> targetAngle.getDegrees(), null);
		builder.addDoubleProperty("Distance To Speaker", () -> getEstimatedPose().getTranslation().getDistance(new Translation2d(
			DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 16, 5.6)), null);
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

	public Pose2d getEstimatedPose() {
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