/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This is a simple container for math methods which are useful
 */
public class ExtendedMath {

	/**
	 * Clamps an output between {@code min} and {@code max}. "Clamping" refers to restricting a
	 * value between a minimum and a maximum. If the given value is below the minimum, the returned
	 * value is equal to the minimum. If the given value is above the maximum, the returned value is
	 * equal to the maximum. If neither of these conditions are met, the given value is returned as
	 * is.
	 *
	 * @param min    the value minimum
	 * @param max    the value maximum
	 * @param output the value to be clamped
	 * @return the clamped value
	 */
	public static double clamp(double min, double max, double output) {
		return Math.min(max, Math.max(min, output));
	}

	public static double dot(Translation2d a, Translation2d b) {
		return a.getX() * b.getX() + a.getY() * b.getY();
	}

	public static Rotation2d angleBetween(Translation2d a, Translation2d b) {
		return a.getAngle().minus(b.getAngle());
	}

	public static double scalarProjectionOf(Translation2d a, Translation2d b) {
		var norm = b.getNorm();
		if (norm == 0) return 0;
		return dot(a, b) / norm;
	}

	/**
	 * @param translation thing to be normalized
     * @return translation with a magnitude of 1
	 */
	public static Translation2d normalize(Translation2d translation) {
		return translation.div(translation.getNorm());
	}

	public static double withHardDeadzone(double value, double deadzone) {
		if (Math.abs(value) < deadzone) return 0;
		return value;
	}

	public static double withContinuousDeadzone(double input, double slope, double deadzone) {
		if (input <= -deadzone) return (input + deadzone) * slope;
		if (-deadzone < input && input < deadzone) return 0;
		return (input - deadzone) * slope;
	}

	public static double withContinuousDeadzone(double input, double deadzone) {
		return withContinuousDeadzone(input, (1 / (1 - deadzone)), deadzone);
	}

	/**
	 * A custom mod function which returns a remainder with the same sign as the dividend. This is
	 * different from using {@code %}, which returns the remainder with the same sign as the
	 * divisor.
	 *
	 * @param a the dividend
	 * @param n the divisor
	 * @return the remainder with the same sign as {@code a}
	 */
	public static double customMod(double a, double n) {
		return a - Math.floor(a / n) * n;
	}

	public static Rotation2d getOptimizedAngleDifference(
		Rotation2d currentAngle,
		Rotation2d targetAngle
	) {
		double wrappedCurrent = wrapRotation2d(currentAngle).getDegrees();
		double wrappedTarget = wrapRotation2d(targetAngle).getDegrees();
		double originalDifference = wrappedTarget - wrappedCurrent;
		double alternateDiffernece = (360 - originalDifference) * -Math.signum(originalDifference);
		if (Math.abs(originalDifference) < Math.abs(alternateDiffernece)) {
			return Rotation2d.fromDegrees(originalDifference);
		}
		return Rotation2d.fromDegrees(alternateDiffernece);
	}

	/**
	 * Uses bigsad Euler Angles math to get the overall angle of the robot.
	 * @param yaw the yaw angle in radians - Rotation about the Z axis
	 * @param pitch the pitch angle in radians - Rotation about the X axis
	 * @param roll the roll angle in radians - Rotation about the Y axis
	 * @return the overall angle of the robot in radians
	 */
	public static double getOverallAngle(
		double yaw,
		double pitch,
		double roll
	) {
		// Set the ground plane normal vector
		double[] n1 = { 0.0, 0.0, 1.0 };

		// Calculate the rotation matrices
		double[][] Rx = {
			{ 1.0, 0.0, 0.0 },
			{ 0.0, Math.cos(pitch), -Math.sin(pitch) },
			{ 0.0, Math.sin(pitch), Math.cos(pitch) },
		};

		double[][] Ry = {
			{ Math.cos(roll), 0.0, Math.sin(roll) },
			{ 0.0, 1.0, 0.0 },
			{ -Math.sin(roll), 0.0, Math.cos(roll) },
		};

		double[][] Rz = {
			{ Math.cos(yaw), -Math.sin(yaw), 0.0 },
			{ Math.sin(yaw), Math.cos(yaw), 0.0 },
			{ 0.0, 0.0, 1.0 },
		};

		// Calculate the normal vector of the rotated plane
		double[] n2 = new double[3];
		for (int i = 0; i < 3; i++) {
			n2[i] = 0.0;
			for (int j = 0; j < 3; j++) {
				n2[i] += Rz[i][j] * Ry[j][i] * Rx[j][i] * n1[j];
			}
		}

		// Calculate the dot product of the two normal vectors
		double dotProduct = 0.0;
		for (int i = 0; i < 3; i++) {
			dotProduct += n1[i] * n2[i];
		}

		// Calculate the overall tilt angle in degrees
		return Math.acos(dotProduct);
	}

	public static boolean isClose(
		Pose2d pose1,
		Pose2d pose2,
		Pose2d threshold
	) {
		Pose2d poseDiff = pose1.relativeTo(pose2);
		boolean xClose = Math.abs(poseDiff.getX()) < Math.abs(threshold.getX());
		boolean yClose = Math.abs(poseDiff.getY()) < Math.abs(threshold.getY());
		boolean thetaClose = Math.abs(poseDiff.getRotation().getDegrees()) <
			Math.abs(threshold.getRotation().getDegrees());
		return xClose && yClose && thetaClose;
	}

	public static double singedSquare(double input) {
		return Math.signum(input) * Math.pow(input, 2);
	}

	public static double cubicLinear(double input, double a, double b) {
		return (a * Math.pow(input, 3) + b * input);
	}

	public static Rotation2d wrapRotation2d(Rotation2d rotationToWrap) {
		return Rotation2d.fromRadians(MathUtil.angleModulus(rotationToWrap.getRadians()));
	}

	/**
	 * @param desiredState the target state of the module
	 * @param currentAngle the current angle of the module
	 * @param continuousRotation whether the encoder of the angle motor of the module 
	 * supports continous rotation
	 * @see <a
	 *      href=https://www.chiefdelphi.com/t/swerve-modules-flip-180-degrees-periodically-conditionally/393059/3
	 *      >Chief Delphi Post Concerning The Issue</a>
	 */
	public static SwerveModuleState optimizeModuleState(
		SwerveModuleState desiredState,
		Rotation2d currentAngle,
		boolean continuousRotation
	) {
		if (continuousRotation) {
			return SwerveModuleState.optimize(desiredState, currentAngle);
		}
		double originalAngle = currentAngle.getDegrees();
		double delta = MathUtil.inputModulus(
			desiredState.angle.getDegrees() - originalAngle + 180, 0, 360
		) - 180;
		if (Math.abs(delta) > 90) {
			return new SwerveModuleState(
				-desiredState.speedMetersPerSecond,
				Rotation2d.fromDegrees(
					originalAngle + delta - Math.signum(delta) * 180
				)
			);
		}
		return new SwerveModuleState(
			desiredState.speedMetersPerSecond,
			Rotation2d.fromDegrees(originalAngle + delta)
		);
	}
}
