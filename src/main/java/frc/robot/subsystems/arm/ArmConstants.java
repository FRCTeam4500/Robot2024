package frc.robot.subsystems.arm;

/**
 * @author Max
 * @author Yijia
 */
public class ArmConstants {

    public static final int TILT_MOTOR_ID = 123; // shooter tilt
    public static final int EXTENSION_MOTOR_ID = 321; // arm extension
    public static final double extensionPositionThreshold = 1;
    public static final double tiltPositionThreshold = 1.1;

    /**
     * @param displacement distance the arm has extended
     * @param maxDisplace
     */
    public static enum ArmState {
        ZERO(0.0, 1.2),
        SPEAKER(1.0, 1.1),
        AMP(2.0,1.3),
        HANDOFF(3.1,1.2);
        public double tiltPosition;

        public double extensionPosition;

        private ArmState(double extensionPosition, double tiltPosition) {
            this.extensionPosition = extensionPosition;
            this.tiltPosition = tiltPosition;
        }

    }

}
