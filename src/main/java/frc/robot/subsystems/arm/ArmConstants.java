package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * @author Max
 * @author Yijia
 * @author Oliver
 */
public class ArmConstants {

    public static final double extensionThreshold = 75;

    public static enum ArmState {
        ZERO(0.0),
        SPEAKER(0.0),
        AMP(5100),
        HANDOFF(3100);
        public double extension;
        private ArmState(double extensionPosition) {
            this.extension = extensionPosition;
        }
    }

}
