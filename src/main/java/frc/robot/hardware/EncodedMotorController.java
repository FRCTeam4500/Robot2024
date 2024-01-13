package frc.robot.hardware;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncodedMotorController {
	/** @param velocity Angular velocity per second */
	public void setAngularVelocity(Rotation2d velocity);
	/** @return angular velocity per second */
	public Rotation2d getAngularVelocity();
	/** @param angle Angle*/
	public void setAngle(Rotation2d angle);
	/** @return angle*/
	public Rotation2d getAngle();
	/** @param output Percent output between -1 and 1 */
	public void setOutput(double output);
	/** return percent output between -1 and 1 */
	public double getOutput();
	/** @return whether the controller supports continuous rotation */
	public boolean hasContinuousRotation();
	/** @param limitAmps The new current limit in amps */
	public EncodedMotorController configCurrentLimit(int limtAmps);
	/** @param pid The new PID gains */
	public EncodedMotorController configAnglePID(PIDConstants pid);
	/** @param pid The new PID gains */
	public EncodedMotorController configVelocityPID(PIDConstants pid);
	/** @param min The minimum angle of the motor */
	public EncodedMotorController configMinAngle(Rotation2d min);
	/** @param max The maximum angle of the motor */
	public EncodedMotorController configMaxAngle(Rotation2d max);
	/** @param minOutput The minimum percent output of the motor */
	public EncodedMotorController configMinOutput(double minOutput);
	/** @param maxOutput The maximum percent output of the motor */
	public EncodedMotorController configMaxOutput(double maxPercentOutput);
	/** @param shouldInvert Whether the motor should be inverted */
	public EncodedMotorController configInverted(boolean shouldInvert);
	/** @param shouldBreak Whether the motor should break when no voltage is applied */
	public EncodedMotorController configBrakeOnIdle(boolean shouldBreak);
}