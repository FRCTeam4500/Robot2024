package frc.robot.subsystems.swerve.real;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
	private SwerveMotor driveMotor;
	private SwerveMotor angleMotor;
	private SwerveModuleConfig config;

	public SwerveModule(
		SwerveMotor driveMotor,
		SwerveMotor angleMotor,
		Translation2d translationToCenter,
		double wheelDiameter
	) {
		this.driveMotor = driveMotor;
		this.angleMotor = angleMotor;
	}

	public SwerveModule(
		SwerveMotor driveMotor,
		SwerveMotor angleMotor,
		SwerveModuleConfig config
	) {
		this.driveMotor = driveMotor;
		this.angleMotor = angleMotor;
		this.config = config;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = SwerveModuleState.optimize(
			initialTargetState, 
			getModuleState().angle
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
			driveMotor.getAngularVelocity()
				.times(config.driveRatio())
				.times(Math.PI)
				.times(config.wheelDiameter())
				.getRotations(),
			new Rotation2d(angleMotor.getAngle().getRadians() * config.angleRatio())
		);
	}

	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity().getRadians() * config.angleRatio();
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle().getRadians() /
			(2 * Math.PI) * 
			config.driveRatio() *
			config.wheelDiameter() *
			Math.PI,
			getModuleState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return config.translation();
	}

	public void setModuleAngle(double targetAngleRadians) {
		angleMotor.setAngle(new Rotation2d(targetAngleRadians / config.angleRatio()));
	}

	public void setModuleVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			new Rotation2d(targetVelocityMetersPerSecond * 2 /
			(config.driveRatio() * config.wheelDiameter()))
		);
		driveMotor.setAngularVelocity(Rotation2d.fromRotations(
			targetVelocityMetersPerSecond / config.wheelDiameter() / Math.PI / config.driveRatio()	
		));
	}

	public static record SwerveModuleConfig(Translation2d translation, double wheelDiameter, double driveRatio, double angleRatio) {}
}