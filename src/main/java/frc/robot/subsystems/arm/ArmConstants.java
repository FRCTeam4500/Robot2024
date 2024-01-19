package frc.robot.subsystems.arm;

/**
 * @author Max
 * @author Yijia
 */
public class ArmConstants {

    public static final int TILT_MOTOR_ID = 123; // shooter tilt
    public static final int EXTENSION_MOTOR_ID = 321; // arm extension
    public static final double extensionPositionThreshold = 1;

    /**
     * @param displacement distance the arm has extended
     * @param maxDisplace
     */
    public static enum ArmState {
        ZERO(0.0),
        SPEAKER(1.0),
        AMP(2.0);

        public double position;

        private ArmState(double position) {
            this.position = position;
            hi
        }

    }

}
