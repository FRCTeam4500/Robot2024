package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.utilities.ExtendedMath;

public class SwerveModule {
	private EncodedMotorController driveMotor;
	private EncodedMotorController angleMotor;
	private Translation2d translationFromCenter;

	public SwerveModule(
		EncodedMotorController driveMotor,
		EncodedMotorController angleMotor,
		Translation2d translationToCenter
	) {
		this.driveMotor = driveMotor;
		this.angleMotor = angleMotor;
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = ExtendedMath.optimizeModuleState(
			initialTargetState, 
			getModuleState().angle,
			angleMotor.hasContinuousRotation()
		);
		setModuleVelocity(
			targetState.speedMetersPerSecond * 
            // Scale velocity by how far wheel is from target
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle.getRadians());
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity().getRadians() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER_METERS /
			2,
			new Rotation2d(angleMotor.getAngle().getRadians() * SwerveConstants.ANGLE_RATIO)
		);
	}

	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity().getRadians() * SwerveConstants.ANGLE_RATIO;
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle().getRadians() /
			(2 * Math.PI) * 
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER_METERS *
			Math.PI,
			getModuleState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	public void setModuleAngle(double targetAngleRadians) {
		angleMotor.setAngle(new Rotation2d(targetAngleRadians / SwerveConstants.ANGLE_RATIO));
	}

	public void setModuleVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			new Rotation2d(targetVelocityMetersPerSecond * 2 /
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER_METERS))
		);
	}
}