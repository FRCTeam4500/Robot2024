/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.utilities.ExtendedMath;

public class NavX {
	private AHRS ahrs;
	private Rotation2d gyroZero;

	public NavX(edu.wpi.first.wpilibj.I2C.Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = new Rotation2d();
	}

	public NavX(Port kmxp) {
		ahrs = new AHRS(kmxp);
		gyroZero = new Rotation2d();
	}

	public AHRS getAHRS() {
		return ahrs;
	}

	public Rotation2d getUnwrappedAngle() {
		return ahrs.getRotation2d();
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getAngle() {
		return getYaw();
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getOffsetedAngle() {
		return ExtendedMath.wrapRotation2d(getAngle().minus(getZero()));
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(-ahrs.getYaw());
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getPitch() {
		return Rotation2d.fromDegrees(-ahrs.getPitch());
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getRoll() {
		return Rotation2d.fromDegrees(-ahrs.getRoll());
	}

	/** Interval: [-pi, pi] */
	public Rotation2d getZero() {
		return gyroZero;
	}

	public void zero() {
		gyroZero = getAngle();
	}

	/** @param offset The new angle given by {@link frc.robot.hardware.NavX#getOffsetedAngle() 
	 * getOffsetedAngle()} for the current angle
	 */
	public void zeroWithOffset(Rotation2d offset) {
		gyroZero = ExtendedMath.wrapRotation2d(getAngle().minus(offset));
	}
}
