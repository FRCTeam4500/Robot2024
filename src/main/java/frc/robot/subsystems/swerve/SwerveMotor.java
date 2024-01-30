package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveMotor {
    public void setAngle(Rotation2d target);
    public void setAngularVelocity(Rotation2d target);
    public Rotation2d getAngle();
    public Rotation2d getAngularVelocity();
    public boolean hasContinuousRotation();
}
