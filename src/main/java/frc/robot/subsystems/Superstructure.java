package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.CommandArm;
import frc.robot.subsystems.climber.CommandClimber;
import frc.robot.subsystems.intake.CommandIntake;
import frc.robot.subsystems.shooter.CommandShooter;
import frc.robot.subsystems.swerve.CommandSwerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Superstructure {
    private static Superstructure instance;
    public static synchronized Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }

    private CommandSwerve swerve;
    private CommandIntake intake;
    private CommandArm arm;
    private CommandShooter shooter;
    private CommandClimber climber;
    private DriveMode driveMode;

    public Superstructure() {
        swerve = CommandSwerve.getInstance();
        intake = CommandIntake.getInstance();
        arm = CommandArm.getInstance();
        shooter = CommandShooter.getInstance();
        driveMode = DriveMode.AngleCentric;
        climber = CommandClimber.getInstance();
        configurePathPlanner();

        Shuffleboard.getTab("Display").add(swerve);
        Shuffleboard.getTab("Display").addString("Drive Mode", () -> driveMode.toString());
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
        NamedCommands.registerCommand("Drive To Piece", swerve.driveToPiece().withTimeout(1.5));
    }
    public Command angleCentricDrive(CommandXboxController xbox) {
        return swerve.angleCentricDrive(xbox);
    }

    public Command robotCentricDrive(CommandXboxController xbox) {
        return swerve.robotCentricDrive(xbox);
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

    public Command resetGyro() {
        return swerve.resetGyro();
    }

    public Command startIntake() {
        return intake.startPickup();
    }

    public Command finishIntake() {
        return arm.goToHandoffCommand().andThen(
            shooter.load()
        ).andThen(
            intake.handoff()
        );
    }

    public Command shoot() {
        return null;
    }
    
    public Command shootInSpeaker() {
        return arm.goToSpeakerCommand().andThen(shooter.shoot()).andThen(arm.goToZeroCommand());  
    }
    public Command shootInAmp() {
        return arm.goToAmpCommand().andThen(shooter.shoot()).andThen(arm.goToZeroCommand());
    } 
    public Command climbUp() {
        return climber.readyClimb();
    }
    public Command pullUp() {
        return climber.climb();
    }

    // shoot: Shoots
    // readySpeaker: get arm ready to fire at speaker, spin up shooter
    // readyAmp: get arm ready to fire at amp, spin up shooter
    // climberUp: get ready to climb
    // climberDown: actually climb
    // intakeStart: start intaking
    // intakeFinish: stop intaking
    
}