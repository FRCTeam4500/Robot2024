package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    /** How far the intake tilt can be from the target to be counted as having reached it */
    public static final Rotation2d tiltThreshold = Rotation2d.fromDegrees(0);

    /** Contains all the states the intake could be at */
    public static enum IntakeState {
        /** The intake is down, but not running */
        ReadyPickup(Rotation2d.fromDegrees(50), 0),
        /** The intake is down and running */
        ExecutePickup(Rotation2d.fromDegrees(50), -0.5),
        /** The intake is zeroed and not running */
        Stow(Rotation2d.fromDegrees(0), 0),
        /** The intake is at the handoff position, but not running */
        ReadyHandoff(Rotation2d.fromDegrees(10), 0),
        /** The intake is at the handoff position and running */
        ExecuteHandoff(Rotation2d.fromDegrees(10), 0.5);

        public Rotation2d tilt;
        public double output;
        private IntakeState(Rotation2d tilt, double output) {
            this.tilt = tilt;
            this.output = output;
        }
    }
}
