package frc.robot.subsystems.tagVision.sim;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.tagVision.AprilTagVisionIO;

public class AprilTagVisionSim extends AprilTagVisionIO {
    @Override
    public void toLog(LogTable table) {
        table.put("Sees tag", seesTag());
        table.put("Tag ID", getTagId(0));
        Logger.recordOutput("Vision Robot Pose", getRobotPose(new Pose2d()));
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public int getTagId(int defaultId) {
        return defaultId;
    }

    @Override
    public boolean seesTag() {
       return false;
    }

    @Override
    public Pose2d getRelativeTagPose(Pose2d defaultPose) {
        return defaultPose;
    }

    @Override
    public Pose2d getRobotPose(Pose2d defaultPose) {
        return defaultPose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Sees Tag", () -> seesTag(), null);
        builder.addIntegerProperty("Target Tag", () -> getTagId(0), null);
        builder.addDoubleProperty("X Pose", () -> getRobotPose(new Pose2d()).getX(), null);
    }
    
}
