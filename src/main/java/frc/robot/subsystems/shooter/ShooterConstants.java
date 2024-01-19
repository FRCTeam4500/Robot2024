package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final double shooterZero = 0;
    public static final double shooterShooting = 1;
    public static final double loaderZero = 0;
    public static final double loaderLoading = 1;
    public static final double loaderShooting = 0.5;
    public static final double threshold = 0.07;

    public static enum ShooterState {
        Loading(shooterZero, loaderLoading),
        Off(shooterZero, loaderZero),
        Shooting(shooterShooting, loaderLoading),
        SpinningUp(shooterShooting, loaderZero);

        public double shooterSpeed;
        public double loaderSpeed;

        private ShooterState(double shooterMotorSpeed, double loaderMotorSpeed) {
            shooterSpeed = shooterMotorSpeed;
            loaderSpeed = loaderMotorSpeed;
        }
    }
}
