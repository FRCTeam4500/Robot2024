package frc.robot.subsystems.climber;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * @author Gasya
 * @author Ashwin
 */
public class ClimberConstants {

public static final int LEFTMOTORID = 100;
public static final int RIGHTMOTORID = 100;

public static enum ClimberState{
    Highhook(Rotation2d.fromDegrees(100)),
    Lowhook(Rotation2d.fromDegrees(0)),
    Midhook(Rotation2d.fromDegrees(50));

    public Rotation2d tilt;
    private ClimberState(Rotation2d tilt){
        this.tilt=tilt;
    }

}
}
