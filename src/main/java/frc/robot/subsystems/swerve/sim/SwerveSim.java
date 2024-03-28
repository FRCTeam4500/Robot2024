package frc.robot.subsystems.swerve.sim;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.utilities.ExtendedMath;

public class SwerveSim extends SwerveIO {
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;
    private PIDController anglePID;
    private SwerveDriveKinematics kinematics;
    private Rotation2d targetAngle;
    private Field2d field;
    public SwerveSim() {
        anglePID = new PIDController(5, 0, 0);
		anglePID.enableContinuousInput(-Math.PI, Math.PI);
		anglePID.setTolerance(Math.PI / 32, Math.PI / 32);
		anglePID.setSetpoint(0);
        kinematics = new SwerveDriveKinematics(
            FRONT_LEFT_MODULE_TRANSLATION,
            FRONT_RIGHT_MODULE_TRANSLATION,
            BACK_LEFT_MODULE_TRANSLATION,
            BACK_RIGHT_MODULE_TRANSLATION
        );
        currentPose = new Pose2d();
        currentSpeeds = new ChassisSpeeds();
        field = new Field2d();
        field.setRobotPose(new Pose2d());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty("Front Left Angle", () -> getModuleStates()[0].angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> getModuleStates()[0].speedMetersPerSecond, null);
                builder.addDoubleProperty("Front Right Angle", () -> getModuleStates()[1].angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> getModuleStates()[1].speedMetersPerSecond, null);
                builder.addDoubleProperty("Back Left Angle", () -> getModuleStates()[2].angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> getModuleStates()[2].speedMetersPerSecond, null);
                builder.addDoubleProperty("Back Right Angle", () -> getModuleStates()[3].angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> getModuleStates()[3].speedMetersPerSecond, null);
            }
        });
        targetAngle = new Rotation2d();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            currentSpeeds = new ChassisSpeeds();
        }
        var requestedStates = kinematics.toSwerveModuleStates(currentSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(requestedStates, MAX_LINEAR_SPEED_MPS);
        currentSpeeds = kinematics.toChassisSpeeds(requestedStates);

        ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, getEstimatedPose().getRotation());
        currentPose = new Pose2d(
            currentPose.getX() + 0.02 * fieldCentricSpeeds.vxMetersPerSecond,
            currentPose.getY() + 0.02 * fieldCentricSpeeds.vyMetersPerSecond,
            Rotation2d.fromRadians(currentPose.getRotation().getRadians() + 0.02 * fieldCentricSpeeds.omegaRadiansPerSecond)
        );
        field.setRobotPose(currentPose);
    }

    @Override
    public void toLog(LogTable table) {
        Logger.recordOutput("Module States", getModuleStates());
        Logger.recordOutput("Estimated Pose", getEstimatedPose());
        Logger.recordOutput("Current Speeds", getChassisSpeeds());
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public Command angleCentricDrive(CommandXboxController xbox) {
        return Commands.run(() -> {
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
        ).beforeStarting(
            Commands.runOnce(() -> targetAngle = getEstimatedPose().getRotation())
        );
    }

    @Override
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
        }, this);
    }

    @Override
    public Command poseCentricDrive(Pose2d target) {
        return AutoBuilder.pathfindToPoseFlipped(target, TELEOP_CONSTRAINTS);
	}

    @Override
    public Command resetGyro() {
        return Commands.runOnce(
            () -> {
				if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
					targetAngle = Rotation2d.fromDegrees(180);
				} else {
					targetAngle = new Rotation2d();
				}
                currentPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            }
        );
    }

    @Override
    public Pose2d getEstimatedPose() {
        return currentPose;
    }

    @Override
    public void resetPose(Pose2d pose) {
        currentPose = pose;
        targetAngle = pose.getRotation();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return currentSpeeds;
    }

    @Override
    public void driveRobotCentric(ChassisSpeeds speeds) {
        currentSpeeds = speeds;
    }

    @Override
    public void driveAngleCentric(double forward, double sideways, Rotation2d targetAngle) {
        Rotation2d addition = new Rotation2d();
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			addition = Rotation2d.fromDegrees(180);
		}
		driveRobotCentric(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forward,
				sideways,
				calculateRotationalVelocityToTarget(targetAngle),
				currentPose.getRotation().plus(addition)
			)
		);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Forward Velocity (mps)", () -> getChassisSpeeds().vxMetersPerSecond, null);
		builder.addDoubleProperty("Sideways Velocity (mps)", () -> getChassisSpeeds().vyMetersPerSecond, null);
		builder.addDoubleProperty("Rotational Velocity (radps)", () -> getChassisSpeeds().omegaRadiansPerSecond, null);
		builder.addDoubleProperty("Distance To Speaker", () -> getEstimatedPose().getTranslation().getDistance(new Translation2d(
			DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 16, 5.6)), null);
    }

    private double calculateRotationalVelocityToTarget(Rotation2d targetRotation) {
		double rotationalVelocity = anglePID.calculate(
			currentPose.getRotation().getRadians(),
			targetRotation.getRadians()
		);
		if (anglePID.atSetpoint()) {
			rotationalVelocity = 0;
		}
		return rotationalVelocity;
	}
    
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			ChassisSpeeds.discretize(currentSpeeds, 0.02)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			MAX_LINEAR_SPEED_MPS
		);
        return states;
    }
}
