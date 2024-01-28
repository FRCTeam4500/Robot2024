package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * @author Max
 * @author Yijia
 * @author Oliver
 */
public class ArmConstants {

    public static final Rotation2d extensionThreshold = Rotation2d.fromDegrees(1);

    public static enum ArmState {
        ZERO(Rotation2d.fromDegrees(0.0)),
        SPEAKER(Rotation2d.fromDegrees(1.0)),
        AMP(Rotation2d.fromDegrees(2.0)),
        HANDOFF(Rotation2d.fromDegrees(3.1));
        public Rotation2d extension;
        private ArmState(Rotation2d extensionPosition) {
            this.extension = extensionPosition;
        }

    }

}
