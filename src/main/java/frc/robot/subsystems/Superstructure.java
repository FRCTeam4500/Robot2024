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
import frc.robot.subsystems.climber.Climber;
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
    private Climber climber;

    public Superstructure() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        telescope = Telescope.getInstance();
        shooter = Shooter.getInstance();
        climber = Climber.getInstance();
        configurePathPlanner();
    }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            swerve::getEstimatorPose,
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
        NamedCommands.registerCommand(
            "Subwoofer Shoot + Intake",
            shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
                .andThen(telescope.extend(Telescope.SUBWOOFER))
                .andThen(shooter.pivot(Shooter.HANDOFF_TILT))
                .andThen(intake.zeroIntake().raceWith(Commands.waitSeconds(0.8)))
                .andThen(shooter.pivot(Shooter.SUBWOOFER_TILT))
                .andThen(intake.tilt(Intake.GROUND_TILT))
                .andThen(intake.run(Intake.PICKUP_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(telescope.extend(Telescope.STAGE))
                .andThen(shooter.pivot(Shooter.STAGE_TILT))
        );
        NamedCommands.registerCommand(
            "Finish Intake + Far Shot + Intake",
            intake.zeroIntake().raceWith(Commands.waitSeconds(1.512))
                .andThen(intake.run(Intake.SHOOTING_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(intake.tilt(Intake.GROUND_TILT))
                .andThen(intake.run(Intake.PICKUP_SPEED))
        );
        NamedCommands.registerCommand(
            "Finish Intake + Far Shot",
            intake.tilt(Intake.HANDOFF_TILT)
                .andThen(telescope.extend(Telescope.STAGE))
                .andThen(shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED))
                .andThen(shooter.pivot(Shooter.STAGE_TILT))
                .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(intake.run(Intake.SHOOTING_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(stow())
        );
        NamedCommands.registerCommand(
            "Subwoofer Shot + Stow",
            shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
                .andThen(telescope.extend(Telescope.SUBWOOFER))
                .andThen(shooter.pivot(Shooter.HANDOFF_TILT))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.pivot(Shooter.SUBWOOFER_TILT))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(stow())
        );
        NamedCommands.registerCommand(
            "Start Intaking", 
            startIntake()
        );
        NamedCommands.registerCommand(
            "Finish Intaking", 
            stow()
                .andThen(handoff())
        );
    }

    public void debugToShuffleboard() {
        ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        debug.add(swerve);
        debug.add(intake);
        debug.add(telescope);
        debug.add(shooter);
        debug.add(climber);
    }

    public Command shootWithEverything() {
        return intake.tilt(Intake.HANDOFF_TILT)
            .andThen(telescope.extend(Telescope.STAGE))
            .andThen(shooter.spinUp(1, 1))
            .andThen(shooter.load(Shooter.LOADER_SHOOT_SPEED))
            .andThen(shooter.pivot(Shooter.STAGE_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(Intake.HANDOFF_SPEED))
            .andThen(Commands.waitSeconds(1));
    }

    public void setDefaultDrive(CommandXboxController xbox) {
        swerve.setDefaultCommand(xanderDriveCommand(xbox));
    }

    public Command angleCentricDrive(CommandXboxController xbox) {
        return swerve.angleCentricDrive(xbox);
    }

    public Command robotCentricDrive(CommandXboxController xbox) {
        return swerve.robotCentricDrive(xbox);
    }

    public Command fieldCentricDrive(CommandXboxController xbox) {
        return swerve.fieldCentricDrive(xbox);
    }

    public Command xanderDriveCommand(CommandXboxController xbox) {
        return swerve.xanderDrive(xbox);
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
    public Command driveToPose(Pose2d bluePose, Pose2d redPose, double forwardScale, double sidewaysScale) {
        return Commands.run(() -> {
            Pose2d pose = swerve.getEstimatorPose();
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                swerve.driveAngleCentric(
                    ExtendedMath.clamp(-3, 3, (bluePose.getX()  - pose.getX()) * forwardScale),
                    ExtendedMath.clamp(-3, 3, (bluePose.getY() - pose.getY()) * sidewaysScale),
                    bluePose.getRotation());
            } else {
                swerve.driveAngleCentric(
                    ExtendedMath.clamp(-3, 3, (redPose.getX()  - pose.getX()) * forwardScale),
                    ExtendedMath.clamp(-3, 3, (redPose.getY() - pose.getY()) * sidewaysScale),
                    redPose.getRotation());
            }
        }, swerve);
    }

    public Command driveToAmp() {
        return driveToPose(
            new Pose2d(1.9, 7.7, Rotation2d.fromDegrees(-90)), 
            new Pose2d(14.1, 7.7, Rotation2d.fromDegrees(-90)), 
            3, 1
        );
	}

    public Command driveToAmpShot() {
        return driveToPose(
            new Pose2d(1.723, 7.505, Rotation2d.fromRadians(0.775)),
            new Pose2d(14.277, 7.505, Rotation2d.fromRadians(2.366592653689793)),
            2, 3  
        );
    }

    public Command driveToFarShot() {
        return driveToPose(
            new Pose2d(2.3, 5.9, Rotation2d.fromDegrees(0)), 
            new Pose2d(13.7, 5.9, Rotation2d.fromDegrees(180)), // 16 max of field
            2, 3
        );
    }

    public Command resetGyro() {
        return swerve.resetGyro();
    }

    public Command ejectFromIntake() {
        return 
            shooter.pivot(Shooter.HANDOFF_TILT)
            .andThen(Commands.waitSeconds(0.25))
            .andThen(intake.tilt(Intake.GROUND_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(Intake.HANDOFF_SPEED));
    }

    public Command zeroIntake() {
        return shooter.pivot(Shooter.HANDOFF_TILT)
        .andThen(Commands.waitSeconds(0.25))
        .andThen(intake.zeroIntake());
    }

    public Command readyAmp() {
        return telescope.extend(Telescope.AMP)
            .andThen(shooter.spinUp(Shooter.AMP_SPEED, Shooter.AMP_SPEED))
            .andThen(shooter.pivot(Shooter.AMP_TILT));
    }

    public Command readyShoot() {
        return shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
            .andThen(telescope.extend(Telescope.SUBWOOFER))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.pivot(Shooter.SUBWOOFER_TILT));
    }

    public Command handoff() {
        return intake.tilt(Intake.HANDOFF_TILT)
            .andThen(shooter.pivot(Shooter.HANDOFF_TILT))
            .andThen(shooter.load(Shooter.LOADER_HANDOFF_SPEED))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(Intake.HANDOFF_SPEED))
            .andThen(Commands.waitSeconds(1))
            .andThen(shooter.load(0.25))
            .andThen(shooter.spinUp(-0.15, -0.15 ))
            .andThen(Commands.waitSeconds(0.15))
            .andThen(shooter.load(0))
            .andThen(shooter.spinUp(0, 0))
            .andThen(intake.run(Intake.OFF_SPEED));
    }

    public Command startIntake() {
        return shooter.pivot(Shooter.HANDOFF_TILT)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.tilt(Intake.GROUND_TILT)
            .andThen(intake.run(Intake.PICKUP_SPEED))
        );
    }

    public Command shoot() {
        return shooter.load(-1)
            .andThen(Commands.waitSeconds(0.9))
            .andThen(shooter.load(Shooter.LOADER_OFF_SPEED));
    }

    public Command stow() {
        return intake.tilt(Intake.STOW_TILT)
            .andThen(intake.run(Intake.OFF_SPEED))
            .andThen(telescope.coast())
            .andThen(climber.extend(Climber.ZERO))
            .andThen(shooter.spinUp(Shooter.OFF_SPEED, Shooter.OFF_SPEED))
            .andThen(shooter.load(Shooter.LOADER_OFF_SPEED))
            .andThen(Commands.waitSeconds(0.75))
            .andThen(shooter.pivot(Shooter.STOW_TILT))
            .andThen(Commands.waitSeconds(0.25))
            .andThen(shooter.coast());
            
    }

    public Command readyFarShot() {
        return shooter.spinUp(Shooter.SUBWOOFER_LEFT_SPEED, Shooter.SUBWOOFER_RIGHT_SPEED)
            .andThen(telescope.extend(Telescope.SUBWOOFER))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.pivot(Shooter.STAGE_TILT));
    }

    public Command confirmIntake() {
        return intake.run(Intake.PICKUP_SPEED);
    }

    public Command offIntake() {
        return intake.run(Intake.OFF_SPEED);
    }

    public Command backOut() {
        return shooter.load(0.25)
            .andThen(shooter.spinUp(-0.15, -0.15 ))
            .andThen(Commands.waitSeconds(0.15))
            .andThen(shooter.load(0))
            .andThen(shooter.spinUp(0, 0));
    }

    public Command climberUp() {
        return shooter.pivot(Shooter.AMP_TILT)
            .andThen(intake.tilt(Intake.GROUND_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(climber.extend(Climber.EXTENDED));
    }

    public Command climberDown() {
        return climber.extend(Climber.RETRACTED)
            .andThen(Commands.waitSeconds(2))
            .andThen(shooter.pivot(Shooter.STOW_TILT));
    }
}