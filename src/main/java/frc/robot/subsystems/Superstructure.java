package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
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
    private Telescope arm;
    private Shooter shooter;

    public Superstructure() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        arm = Telescope.getInstance();
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
                new PIDConstants(5.0),
                new PIDConstants(5.0),
                MAX_LINEAR_SPEED_MPS,
                0.39878808909,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            swerve
        );
        NamedCommands.registerCommand("Drive To Piece", driveToPiece().withTimeout(1.5));
    }

    public void displayToShuffleboard() {
        ShuffleboardTab display = Shuffleboard.getTab("Display");
        display.addBoolean("Gyro", () -> swerve.gyroConnected());
    }

    public void debugToShuffleboard() {
        ShuffleboardTab debug = Shuffleboard.getTab("Debug2");
        debug.add(swerve);
        debug.add(intake);
        debug.add(arm);
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

    public Command resetGyro() {
        return swerve.resetGyro();
    }

    public boolean gyroConnected() {
        return swerve.gyroConnected();
    }

    public Command readyAmp() {
        return arm.extend(Telescope.AMP)
            .andThen(shooter.shoot(Shooter.AMP_SPEED, Shooter.AMP_SPEED))
            .andThen(shooter.pivot(Shooter.AMP_TILT));
    }

    public Command groundIntake() {
        return intake.tilt(Intake.GROUND_TILT)
            .andThen(intake.run(Intake.PICKUP_SPEED));
    }

    public Command handoff() {
        return intake.run(Intake.OFF_SPEED)
            .andThen(intake.tilt(Intake.HANDOFF_TILT))
            .andThen(shooter.pivot(Shooter.HANDOFF_TILT))
            .andThen(shooter.load(Shooter.LOADER_HANDOFF_SPEED))
            .andThen(arm.coast())
            .andThen(Commands.waitSeconds(1.8))
            .andThen(intake.run(Intake.HANDOFF_SPEED))
            .andThen(shooter.waitTillNote().withTimeout(3))
            .andThen(stow());
    }

    public Command readySubwoofer() {
        return arm.extend(Telescope.SUBWOOFER)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.pivot(Shooter.SUBWOOFER_TILT))
            .andThen(shooter.shoot(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED));
    }

    public Command shoot() {
        return shooter.load(-1)
            .andThen(Commands.waitSeconds(3))
            .andThen(shooter.load(Shooter.LOADER_OFF_SPEED));
    }

    public Command stow() {
        return intake.tilt(Intake.STOW_TILT)
            .andThen(intake.run(Intake.OFF_SPEED))
            .andThen(arm.coast())
            .andThen(shooter.shoot(Shooter.OFF_SPEED, Shooter.OFF_SPEED))
            .andThen(shooter.load(Shooter.LOADER_OFF_SPEED))
            .andThen(shooter.pivot(Shooter.STOW_TILT));
    }
}