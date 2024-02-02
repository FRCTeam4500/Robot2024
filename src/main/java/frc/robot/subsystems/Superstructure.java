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
import frc.robot.subsystems.arm.CommandArm;
import frc.robot.subsystems.climber.CommandClimber;
import frc.robot.subsystems.intake.CommandIntake;
import frc.robot.subsystems.shooter2.CommandShooter2;
import frc.robot.subsystems.swerve.CommandSwerve;

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

    private CommandSwerve swerve;
    private CommandIntake intake;
    // private CommandArm arm;
    // private CommandShooter2 shooter;
    // private CommandClimber climber;

    public Superstructure() {
        swerve = CommandSwerve.getInstance();
        intake = CommandIntake.getInstance();
        // arm = CommandArm.getInstance();
        // shooter = CommandShooter2.getInstance();
        // climber = CommandClimber.getInstance();
        // configurePathPlanner();
        displayToShuffleboard();
        debugToShuffleboard();
    }

    /**
     * Configures the path planner for the superstructure.
     * This method sets up the necessary components and parameters for the path planner.
     */
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

    /**
     * Displays the debug information of to Shuffleboard (initSendable)
     */
    public void debugToShuffleboard() {
        ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        debug.add(swerve);
        debug.add(intake);
    }

    /**
     * Sets the default drive command for the superstructure.
     *
     * @param xbox The Xbox controller used for driving.
     */
    public void setDefaultDrive(CommandXboxController xbox) {
        swerve.setDefaultCommand(angleCentricDrive(xbox));
    }

    /**
     * Drives the superstructure in angle-centric mode using the given Xbox controller.
     *
     * @param xbox the Xbox controller used for driving
     * @return the command to drive the superstructure in angle-centric mode
     */
    public Command angleCentricDrive(CommandXboxController xbox) {
        return swerve.angleCentricDrive(xbox);
    }

    /**
     * Aligns the superstructure to a game piece based on input from the Xbox controller.
     *
     * @param xbox The Xbox controller used to control the alignment.
     * @return The command to align the superstructure to the game piece.
     */
    public Command alignToPiece(CommandXboxController xbox) {
        return swerve.alignToPiece(xbox);
    }

    /**
     * Drives the robot to a specified target pose relative to the april tag.
     *
     * @param targetPose The target pose to drive to relative to the april tag.
     * @return The command to drive to the target pose.
     */
    public Command driveToTag(Pose2d targetPose) {
        return swerve.driveToTag(targetPose);
    }

    /**
     * Drives the robot to a piece.
     *
     * @return the command to drive to the piece
     */
    public Command driveToPiece() {
        return swerve.driveToPiece();
    }

    /**
     * Resets the gyro and returns a Command object.
     *
     * @return The Command object that resets the gyro.
     */
    public Command resetGyro() {
        return swerve.resetGyro();
    }

    // public boolean gyroConnected() {
    //     return swerve.gyroConnected();
    // }

    public Command runIntake() {
        return Commands.runOnce(
            () -> intake.setTilt(Rotation2d.fromRotations(-15))
        ).alongWith(
            Commands.runOnce(() -> intake.setSpeed(0.3))
        ).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            Commands.runOnce(() -> intake.coastTilt())
        );
    }

    public Command offIntake() {
        return Commands.runOnce(
            () -> intake.setTilt(Rotation2d.fromRotations(-2))
        ).alongWith(
            Commands.runOnce(() -> intake.setSpeed(0))
        ).andThen(
            Commands.waitSeconds(1.5)
        ).andThen(
            Commands.runOnce(() -> intake.coastTilt())
        );
    }

    public Command ejectIntake() {
        return Commands.startEnd(
            () -> intake.setSpeed(-0.4),
            () -> intake.setSpeed(0)  
        );
    }

    // /**
    //  * Start the intake to pick up a game piece.
    //  */
    // public Command startPickUp() {
    //     return intake.startPickup();
    // }

    // /**
    //  * Returns a Command object that represents the end of the pick-up process.
    //  * This command is composed of two sequential commands: readyHandoff() and eject().
    //  *
    //  * @return The Command object representing the end of the pick-up process.
    //  */
    // public Command endPickUp() {
    //     return intake.readyHandoff().andThen(intake.eject());
    // // }

    // /**
    //  * Returns a Command object that represents the action of moving the climber up.
    //  *
    //  * @return the Command object for moving the climber up
    //  */
    // public Command climberUp() {
    //     return climber.readyClimb();
    // }

    // /**
    //  * Returns a command to initiate the climber's descent.
    //  *
    //  * @return the command to initiate the climber's descent
    //  */
    // public Command climberDown() {
    //     return climber.climb();
    // }

    // /**
    //  * Executes the shoot command.
    //  *
    //  * @return the shoot command
    //  */
    // public Command shoot() {
    //     return shooter.shoot();
    // }

    // /**
    //  * Returns a Command object that represents the action of getting the shooter ready.
    //  *
    //  * @return a Command object representing the action of getting the shooter ready
    //  * @author sal and gre
    //  * @param sal
    //  */
    // public Command readyShooter() {
    //     return shooter.readySpeaker();
    // }

    // /**
    //  * Returns a Command object that represents the action of getting ready for amplification.
    //  * This command is composed of two sub-commands: one for the arm to go to the amplification position,
    //  * and one for the shooter to get ready for amplification.
    //  *
    //  * @return The Command object representing the action of getting ready for amplification.
    //  */
    // public Command readyAmp() {
    //     return arm.goToAmpCommand().alongWith(shooter.readyAmp());//ppap
    // }

}