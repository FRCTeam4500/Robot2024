package frc.robot.subsystems.tagVision.sim;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.tagVision.AprilTagVisionIO;

public class AprilTagVisionSim extends AprilTagVisionIO {
    @Override
    public void toLog(LogTable table) {
        table.put("Front Sees Tag", seesTag(Camera.Front));
        table.put("Front Tag ID", getTagId(0, Camera.Front));
        Logger.recordOutput("Front Vision Robot Pose", getRobotPose(new Pose2d(), Camera.Front));
        table.put("Back Sees Tag", seesTag(Camera.Back));
        table.put("Back Tag ID", getTagId(0, Camera.Back));
        Logger.recordOutput("Back Vision Robot Pose", getRobotPose(new Pose2d(), Camera.Back));
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public int getTagId(int defaultId, Camera camera) {
        return defaultId;
    }

    @Override
    public boolean seesTag(Camera camera) {
       return false;
    }

    @Override
    public Pose2d getRelativeTagPose(Pose2d defaultPose, Camera camera) {
        return defaultPose;
    }

    @Override
    public Pose2d getRobotPose(Pose2d defaultPose, Camera camera) {
        return defaultPose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Front Sees Tag", () -> seesTag(Camera.Front), null);
        builder.addIntegerProperty("Front Tag ID", () -> getTagId(0, Camera.Front), null);
        builder.addBooleanProperty("Back Sees Tag", () -> seesTag(Camera.Back), null);
        builder.addIntegerProperty("Back Tag ID", () -> getTagId(0, Camera.Back), null);
    }   
}
