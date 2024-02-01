package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * numbers
 * @author David Wharton
 * @author Gretchen Miller
 */
public class ShooterConstants {
   public static final Rotation2d tiltThreshold = Rotation2d.fromDegrees(1);
   public static final double speedThreshold = 0.07;
   public static enum ShooterState {
      Loading(0, 1, Rotation2d.fromDegrees(0)),
      Off(0, 0, Rotation2d.fromDegrees(0)),
      Amp(1, 1, Rotation2d.fromDegrees(0)),
      Subwoofer(1, 1, Rotation2d.fromDegrees(0)),
      LeftStage(1, 1, Rotation2d.fromDegrees(0)),
      RightStage(1, 1, Rotation2d.fromDegrees(0)),
      Podium(1, 1, Rotation2d.fromDegrees(0)),
      SpinningUp(1, 0, Rotation2d.fromDegrees(0));
      public double shooterSpeed;  
      public double loaderSpeed;
      public Rotation2d tilt;
      private ShooterState(double shooterMotorSpeed, double loaderMotorSpeed, Rotation2d tilt) {
         shooterSpeed = shooterMotorSpeed;
         loaderSpeed = loaderMotorSpeed;
         this.tilt = tilt;
      }
   }
}
