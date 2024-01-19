package frc.robot.subsystems.shooter;
/**
 * numbers
 * @author David Wharton
 * @author "lord gre"
 */
public class ShooterConstants {
    /**
 * number when shooter is zerod
 * @author David Wharton
 * @author "lord gre"
 */
    public static final double shooterZero = 0;
    /**
 * number when shooter is shooting
 * @author David Wharton
 * @author "lord gre"
 */
    public static final double shooterShooting = 1;
    /**
 * number when loader is zerod
 * @author David Wharton
 * @author "lord gre"
 */
    public static final double loaderZero = 0;
    /**
 * number when the loader is loading
 * @author David Wharton
 * @author "lord gre"
 */
    public static final double loaderLoading = 1;
    /**
 * number when the loader is existing while the note is being shot
 * @author David Wharton
 * @author "lord gre"
 */
    public static final double loaderShooting = 0.5;
    /**
 * number that tells us when were "close enough"
 * @author David Wharton
 * @author "lord gre"
 */
    public static final double threshold = 0.07;
/**
 * a group of states
 * @author David Wharton
 * @author "lord gre"
 */
    public static enum ShooterState {
        /**
 * loading state
 * @author David Wharton
 * @author "lord gre"
 */
        Loading(shooterZero, loaderLoading),
        /**
 * off state
 * @author David Wharton
 * @author "lord gre"
 */
        Off(shooterZero, loaderZero),
        /**
 * shooting state
 * @author David Wharton
 * @author "lord gre"
 */
        Shooting(shooterShooting, loaderLoading),
        /**
 * spinning up state
 * @author David Wharton
 * @author "lord gre"
 */
        SpinningUp(shooterShooting, loaderZero);
/**
 * speed of shooter
 * @author David Wharton
 * @author "lord gre"
 */
        public double shooterSpeed;
        /**
 * speed of loader
 * @author David Wharton
 * @author "lord gre"
 */
        public double loaderSpeed;
        /**
 * defines the arguments for the states
 * @author David Wharton
 * @author "lord gre"
 */
         //q
        private ShooterState(double shooterMotorSpeed, double loaderMotorSpeed) {
            shooterSpeed = shooterMotorSpeed;
            loaderSpeed = loaderMotorSpeed;
        }
    }
}
