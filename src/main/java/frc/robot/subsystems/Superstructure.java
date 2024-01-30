package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    // private CommandArm arm;
    private CommandShooter shooter;
    private CommandClimber climber;

    public Superstructure() {
        swerve = CommandSwerve.getInstance();
        intake = CommandIntake.getInstance();
        // arm = CommandArm.getInstance();
        shooter = CommandShooter.getInstance();
        climber = CommandClimber.getInstance();
        configurePathPlanner();
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
        // ShuffleboardTab display = Shuffleboard.getTab("Display");
    }

    public void debugToShuffleboard() {
        ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        debug.add(swerve);
        debug.add(intake);
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

    public Command resetGyro() {
        return swerve.resetGyro();
    }

    public Command startPickUp() {
        return intake.startPickup();
    }

    public Command endPickUp() {
        return intake.readyHandoff().andThen(intake.eject());
    }

    public Command climberUp() {
        return climber.readyClimb();
    }

    public Command climberDown() {
        return climber.climb();
    }

    // shoot: Shoots

    public Command shoot() {
        return shooter.shoot();
    }

    // readySpeaker: get arm ready to fire at speaker, spin up shooter
    // readyAmp: get arm ready to fire at amp, spin up shooter

}