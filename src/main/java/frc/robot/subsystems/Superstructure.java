package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.ExtendedMath;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Superstructure class represents the overall structure of the robot.
 * It is responsible for coordinating the different subsystems and commands
 * to perform various actions and tasks.
 *
 * @author Borian Vassilev Schonhuth
 */
public class Superstructure {
    private static Superstructure instance;
    public static synchronized Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }

    private Swerve swerve;
    private Intake intake;
    private Telescope telescope;
    private Shooter shooter;

    public Superstructure() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        telescope = Telescope.getInstance();
        shooter = Shooter.getInstance();
        configurePathPlanner();
        displayToShuffleboard();
        debugToShuffleboard();
    }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            swerve::getOdometryPose,
            swerve::resetPose,
            swerve::getChassisSpeeds,
            swerve::driveRobotCentric,
            new HolonomicPathFollowerConfig(
                new PIDConstants(7.5),
                new PIDConstants(5.0),
                MAX_LINEAR_SPEED_MPS,
                0.39878808909,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            swerve
        );
        NamedCommands.registerCommand("Ready Shot", 
            shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
                .andThen(telescope.extend(Telescope.SPEAKER))
                .andThen(intake.tilt(Intake.HANDOFF_TILT))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
                .andThen(shooter.pivot(Shooter.SPEAKER_TILT))
        );
        NamedCommands.registerCommand("Shoot", 
            intake.run(Intake.SHOOTING_SPEED)
                .andThen(Commands.waitSeconds(.75))
                .andThen(intake.run(Intake.OFF_SPEED))
        );
        NamedCommands.registerCommand("Start Intake",
            intake.tilt(Intake.GROUND_TILT)
                .andThen(intake.run(Intake.PICKUP_SPEED))
        );
        NamedCommands.registerCommand("Finish Intake", 
            intake.run(Intake.OFF_SPEED)
                .andThen(intake.tilt(Intake.HANDOFF_TILT))
        );
        NamedCommands.registerCommand("Stow", 
            stow()
        );
    }

    public Command startShooting() {
        return shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
            .andThen(telescope.extend(Telescope.SPEAKER))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
            .andThen(shooter.pivot(Shooter.SPEAKER_TILT));
    }

    public Command executeShoot() {
        return intake.tilt(Intake.HANDOFF_TILT)
            .andThen(intake.run(Intake.OFF_SPEED))
            .andThen(Commands.waitSeconds(1))
            .andThen(intake.run(Intake.HANDOFF_SPEED))
            .andThen(Commands.waitSeconds(1));
    }

    public void displayToShuffleboard() {
        ShuffleboardTab display = Shuffleboard.getTab("Display");
        display.addBoolean("Gyro", () -> swerve.gyroConnected());
    }

    public void debugToShuffleboard() {
        ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        debug.add(swerve);
        debug.add(intake);
        debug.add(telescope);
        debug.add(shooter);
    }

    public void setDefaultDrive(CommandXboxController xbox) {
        swerve.setDefaultCommand(angleCentricDrive(xbox));
    }

    public Command angleCentricDrive(CommandXboxController xbox) {
        return swerve.angleCentricDrive(xbox);
    }

    public Command alignToPiece(CommandXboxController xbox) {
        return swerve.alignToPiece(xbox);
    }

    public Command driveToTag(Pose2d targetPose) {
        return swerve.driveToTag(targetPose);
    }

    public Command driveToPiece() {
        return swerve.driveToPiece();
    }

    public Command alignToSpeaker(CommandXboxController xbox) {
        return swerve.rotateToSpeaker(xbox);
    }

    public Command driveToAmp() {
		return Commands.run(() -> {
            Pose2d pose = swerve.getEstimatorPose();
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                swerve.driveAngleCentric(
                    ExtendedMath.clamp(-1.5, 1.5, (1.9  - pose.getX()) * 3), 
                    ExtendedMath.clamp(-1.5, 1.5, (7.7 - pose.getY()) * 1), 
                    Rotation2d.fromDegrees(-90));
            } else {
                swerve.driveAngleCentric(0, 0, Rotation2d.fromDegrees(0));
            }
        }, swerve);
	}
    
    public Command resetGyro() {
        return swerve.resetGyro();
    }

    public Command ejectFromIntake() {
        return intake.tilt(Intake.GROUND_TILT)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(Intake.HANDOFF_SPEED));
    }

    public boolean gyroConnected() {
        return swerve.gyroConnected();
    }

    public Command resetIntake() {
        return intake.reset();
    }

    public Command readyAmp() {
        return telescope.extend(Telescope.AMP)
            .andThen(shooter.spinUp(Shooter.AMP_SPEED, Shooter.AMP_SPEED))
            .andThen(shooter.pivot(Shooter.AMP_TILT));
    }

    public Command readyShoot() {
        return shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
            .andThen(telescope.extend(Telescope.SPEAKER))
            .andThen(shooter.pivot(Shooter.SPEAKER_TILT));
    }

    public Command actuallyShoot() {
        return shooter.load(Shooter.LOADER_SHOOT_SPEED)
            .andThen(Commands.waitSeconds(2))
            .andThen(stow());
    }

    public Command groundIntake() {
        return intake.tilt(Intake.GROUND_TILT)
            .andThen(intake.run(Intake.PICKUP_SPEED));
    }

    public Command handoff() {
        return intake.tilt(Intake.HANDOFF_TILT)
            .andThen(shooter.pivot(Shooter.HANDOFF_TILT))
            .andThen(shooter.load(Shooter.LOADER_HANDOFF_SPEED))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(Intake.HANDOFF_SPEED))
            .andThen(Commands.waitSeconds(1.5))
            .andThen(shooter.load(0.25))
            .andThen(shooter.spinUp(-0.15, -0.15 ))
            .andThen(Commands.waitSeconds(0.15))
            .andThen(shooter.load(0))
            .andThen(intake.run(Intake.OFF_SPEED));
    }

    public Command shootWithEverything() {
        return shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
            .andThen(telescope.extend(Telescope.SPEAKER))
            .andThen(shooter.pivot(Shooter.SPEAKER_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
            .andThen(Commands.waitSeconds(1.5))
            .andThen(intake.run(Intake.SHOOTING_SPEED))
            .andThen(Commands.waitSeconds(2))
            .andThen(stow());
    }

    public Command shoot() {
        return shooter.load(-1)
            .andThen(Commands.waitSeconds(1))
            .andThen(shooter.load(Shooter.LOADER_OFF_SPEED));
    }

    public Command stow() {
        return intake.tilt(Intake.STOW_TILT)
            .andThen(intake.run(Intake.OFF_SPEED))
            .andThen(telescope.coast())
            .andThen(shooter.spinUp(Shooter.OFF_SPEED, Shooter.OFF_SPEED))
            .andThen(shooter.load(Shooter.LOADER_OFF_SPEED))
            .andThen(shooter.pivot(Shooter.STOW_TILT))
            .andThen(Commands.waitSeconds(1))
            .andThen(shooter.coast());
    }

    public Command readyShortAmp() {
        return intake.tilt(Intake.AMP_TILT);
    }

    public Command fireShortAmp() {
        return intake.run(Intake.AMP_SPEED)
            .andThen(Commands.waitSeconds(2))
            .andThen(stow());
    }

    public Command confirmIntake() {
        return intake.run(Intake.PICKUP_SPEED);
    }
    
    public Command offIntake() {
        return intake.run(Intake.OFF_SPEED);
    }
}