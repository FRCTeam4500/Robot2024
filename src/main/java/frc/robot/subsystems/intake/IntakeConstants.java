package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static final Rotation2d zeroTilt = Rotation2d.fromDegrees(0);
    public static final Rotation2d handoffTilt = Rotation2d.fromDegrees(10);
    public static final Rotation2d groundTilt = Rotation2d.fromDegrees(50);
    public static final double intakeOutput = -0.5;
    public static final double offOutput = -0.0;
    public static final double ejectingOutput = 0.5;
    public static final Rotation2d positionThreshold = new Rotation2d();
    public static final double outputThreshold = 0;

    public static enum IntakePosition {
        Zero(zeroTilt),
        Handoff(handoffTilt),
        Ground(groundTilt);

        public Rotation2d tilt;

        private IntakePosition(Rotation2d thisTilt) {
            tilt = thisTilt;
        }
    }

    public static enum IntakeSpeed {
        Intaking(intakeOutput),
        Ejecting(ejectingOutput),
        Off(offOutput);

        public double output;

        private IntakeSpeed(double thisOutput) {
            output = thisOutput;
        }
    }
}
