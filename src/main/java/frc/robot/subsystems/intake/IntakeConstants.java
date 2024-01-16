package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
    public static final Rotation2d tiltThreshold = new Rotation2d();

    public static enum IntakeState {
        ReadyPickup(Rotation2d.fromDegrees(50), 0),
        ExecutePickup(Rotation2d.fromDegrees(50), -0.5),
        Stow(Rotation2d.fromDegrees(0), 0),
        ReadyHandoff(Rotation2d.fromDegrees(10), 0),
        ExecuteHandoff(Rotation2d.fromDegrees(10), 0.5);

        public Rotation2d tilt;
        public double output;
        private IntakeState(Rotation2d tilt, double output) {
            this.tilt = tilt;
            this.output = output;
        }
    }
}
