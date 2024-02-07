package frc.robot.subsystems.shooter3;

import edu.wpi.first.math.geometry.Rotation2d;

public class Shooter3Constants {
    public static double LOADING_OUTPUT = 1;
    public static Rotation2d HANDOFF_ANGLE = Rotation2d.fromRotations(10);
    public static Rotation2d READY_HANDOFF_ANGLE = Rotation2d.fromRotations(10);
    public static Rotation2d AMP_ANGLE = Rotation2d.fromRotations(10);
    public static Rotation2d STOW_ANGLE = Rotation2d.fromRotations(0);
    public static double SHOOT_OUTPUT_LEFT = -0.4;
    public static double SHOOT_OUTPUT_RIGHT = -1.0;
}