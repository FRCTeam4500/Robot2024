package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.real.Swerve;
import frc.robot.subsystems.swerve.sim.SwerveSim;

public abstract class SwerveIO extends SubsystemBase implements LoggableInputs {
    private static SwerveIO instance;
    public static synchronized SwerveIO getInstance() {
        if (instance == null && RobotBase.isReal()) instance = new Swerve();
        if (instance == null && !RobotBase.isReal()) instance = new SwerveSim();
        return instance;
    }

    public abstract Command angleCentricDrive(CommandXboxController xbox);
    public abstract Command speakerCentricDrive(CommandXboxController xbox);
    public abstract Command poseCentricDrive(Pose2d target);
    public abstract Command resetGyro();
    public abstract Pose2d getEstimatedPose();
    public abstract void resetPose(Pose2d pose);
    public abstract ChassisSpeeds getChassisSpeeds();
    public abstract void driveRobotCentric(ChassisSpeeds speeds);
    public abstract void driveAngleCentric(double forward, double sideways, Rotation2d targetAngle);
    public abstract void initSendable(SendableBuilder builder);
}
