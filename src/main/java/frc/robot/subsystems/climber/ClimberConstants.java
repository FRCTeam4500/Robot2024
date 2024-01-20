package frc.robot.subsystems.climber;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * @author Gasya
 * @author Ashwin
 */
public class ClimberConstants {
    public static final Rotation2d climberTiltThreshold = Rotation2d.fromDegrees(0);


    public static enum ClimberState {
        /**prapareing for climb */
        Highhook(Rotation2d.fromDegrees(100)),
        /**starting position */
        Lowhook(Rotation2d.fromDegrees(0)),
        /**finishing position*/
        Midhook(Rotation2d.fromDegrees(50));

        public Rotation2d tilt;
        private ClimberState(Rotation2d tilt) {
            this.tilt = tilt;
        }

    }
}
