package frc.robot.subsystems.tagVision;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.tagVision.real.AprilTagVision;
import frc.robot.subsystems.tagVision.sim.AprilTagVisionSim;

public abstract class AprilTagVisionIO extends SubsystemBase implements LoggableInputs {
    private static AprilTagVisionIO instance;
    public static synchronized AprilTagVisionIO getInstance() {
        if (instance == null && RobotBase.isReal()) instance = new AprilTagVision();
        if (instance == null && !RobotBase.isReal()) instance = new AprilTagVisionSim();
        return instance;
    }

    public abstract int getTagId(int defaultId);
    public abstract boolean seesTag();
    public abstract Pose2d getRelativeTagPose(Pose2d defaultPose);
    public abstract Pose2d getRobotPose(Pose2d defaultPose);
    public abstract void initSendable(SendableBuilder builder);
}
