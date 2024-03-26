package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.telescope.TelescopeIO;
import frc.robot.subsystems.tagVision.AprilTagVisionIO;
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

    private static Mechanism2d currentMech;
    public static synchronized Mechanism2d getCurrentMech() {
        if (currentMech == null) {
            currentMech = new Mechanism2d(1.372, 1.2192);
            SmartDashboard.putData("Current Robot State", currentMech);
        }
        return currentMech;
    }

    private static Mechanism2d setpointMech;
    public static synchronized Mechanism2d getSetpointMech() {
        if (setpointMech == null) {
            setpointMech = new Mechanism2d(1.372, 1.2192);
            SmartDashboard.putData("Target Robot State", setpointMech);
        }
        return setpointMech;
    }

    private SwerveIO swerve;
    private IntakeIO intake;
    private TelescopeIO telescope;
    private ShooterIO shooter;
    private ClimberIO climber;

    public Superstructure() {
        swerve = SwerveIO.getInstance();
        intake = IntakeIO.getInstance();
        telescope = TelescopeIO.getInstance();
        shooter = ShooterIO.getInstance();
        climber = ClimberIO.getInstance();
        configurePathPlanner();
        debugToShuffleboard();
        SmartDashboard.putData("To Amp", driveToAmp());
        SmartDashboard.putData("To Amp Shot", driveToAmpShot());
        SmartDashboard.putData("To Far Shot", driveToFarShot());
    }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            swerve::getEstimatedPose,
            swerve::resetPose,
            swerve::getChassisSpeeds,
            swerve::driveRobotCentric,
            new HolonomicPathFollowerConfig(
                new PIDConstants(7.5),
                new PIDConstants(5.0),
                MAX_LINEAR_SPEED_MPS,
                0.39878808909,
                new ReplanningConfig(true, true)
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            swerve
        );
        NamedCommands.registerCommand(
            "Subwoofer Shoot + Intake",
            shooter.spinUp(ShooterIO.SHOOTING_OUTPUT, ShooterIO.SHOOTING_OUTPUT)
                .andThen(telescope.extend(TelescopeIO.SHOOTING))
                .andThen(shooter.pivot(ShooterIO.HANDOFF_TILT))
                .andThen(intake.zero().raceWith(Commands.waitSeconds(0.8)))
                .andThen(shooter.pivot(ShooterIO.SUBWOOFER_TILT))
                .andThen(intake.tilt(IntakeIO.GROUND_TILT))
                .andThen(intake.run(IntakeIO.PICKUP_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.load(ShooterIO.LOADER_SHOOTING_OUTPUT))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(telescope.extend(TelescopeIO.AUTO))
                .andThen(shooter.pivot(ShooterIO.FAR_TILT))
        );
        NamedCommands.registerCommand(
            "Finish Intake + Far Shot + Intake",
            intake.zero().raceWith(Commands.waitSeconds(1.512))
                .andThen(intake.run(IntakeIO.EJECT_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(intake.tilt(IntakeIO.GROUND_TILT))
                .andThen(intake.run(IntakeIO.PICKUP_SPEED))
                .andThen(Commands.waitSeconds(0.25))
        );
        NamedCommands.registerCommand(
            "Finish Intake + Far Shot",
            intake.zero()
                .andThen(telescope.extend(TelescopeIO.AUTO))
                .andThen(shooter.spinUp(ShooterIO.SHOOTING_OUTPUT, ShooterIO.SHOOTING_OUTPUT))
                .andThen(shooter.pivot(ShooterIO.FAR_TILT))
                .andThen(shooter.load(ShooterIO.LOADER_SHOOTING_OUTPUT))
                .andThen(Commands.waitSeconds(1.5))
                .andThen(intake.run(IntakeIO.EJECT_SPEED))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(stow())
        );
        NamedCommands.registerCommand(
            "Subwoofer Shot + Stow",
            shooter.spinUp(ShooterIO.SHOOTING_OUTPUT, ShooterIO.SHOOTING_OUTPUT)
                .andThen(telescope.extend(TelescopeIO.SHOOTING))
                .andThen(shooter.pivot(ShooterIO.HANDOFF_TILT))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.pivot(ShooterIO.SUBWOOFER_TILT))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(shooter.load(ShooterIO.LOADER_SHOOTING_OUTPUT))
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

        NamedCommands.registerCommand(
            "Handoff", 
            handoff()
        );
        NamedCommands.registerCommand(
            "Ready Variable Shot",
            readyVariableShot()
        );
        NamedCommands.registerCommand(
            "Fire",
            shoot()
        );
    }

    public void debugToShuffleboard() {
        ShuffleboardTab debug = Shuffleboard.getTab("Debug");
        debug.add(swerve);
        debug.add(intake);
        debug.add(telescope);
        debug.add(shooter);
        debug.add(climber);
        debug.add(AprilTagVisionIO.getInstance());
    }

    public Command shootWithEverything() {
        return intake.zero()
            .andThen(telescope.extend(TelescopeIO.AUTO))
            .andThen(shooter.spinUp(ShooterIO.SHOOTING_OUTPUT, ShooterIO.SHOOTING_OUTPUT))
            .andThen(shooter.load(ShooterIO.LOADER_SHOOTING_OUTPUT))
            .andThen(shooter.pivot(ShooterIO.FAR_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(IntakeIO.EJECT_SPEED))
            .andThen(Commands.waitSeconds(1));
    }

    public void setDefaultDrive(CommandXboxController xbox) {
        swerve.setDefaultCommand(angleCentricDrive(xbox));
    }

    public Command angleCentricDrive(CommandXboxController xbox) {
        return swerve.angleCentricDrive(xbox);
    }

    public Command alignToSpeaker(CommandXboxController xbox) {
        return swerve.speakerCentricDrive(xbox);
    }
    
    public Command driveToPose(Pose2d target) {
        return swerve.poseCentricDrive(target);
    }

    public Command driveToAmp() {
        return driveToPose(new Pose2d(1.9, 7.7, Rotation2d.fromDegrees(-90)));
	}

    public Command driveToAmpShot() {
        return driveToPose(new Pose2d(1.723, 7.505, Rotation2d.fromRadians(0.775)));
    }

    public Command driveToFarShot() {
        return driveToPose(new Pose2d(2.3, 5.9, Rotation2d.fromDegrees(0)));
    }

    public Command resetGyro() {
        return swerve.resetGyro();
    }

    public Command ejectFromIntake() {
        return 
            shooter.pivot(ShooterIO.HANDOFF_TILT)
            .andThen(Commands.waitSeconds(0.25))
            .andThen(intake.tilt(IntakeIO.GROUND_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(intake.run(IntakeIO.EJECT_SPEED));
    }

    public Command zeroIntake() {
        return shooter.pivot(ShooterIO.HANDOFF_TILT)
        .andThen(Commands.waitSeconds(0.25))
        .andThen(intake.zero());
    }

    public Command readyAmp() {
        return telescope.extend(TelescopeIO.AMP)
            .andThen(shooter.spinUp(ShooterIO.AMPING_OUTPUT, ShooterIO.AMPING_OUTPUT))
            .andThen(shooter.pivot(ShooterIO.AMP_TILT));
    }

    public Command readySubwooferShot() {
        return shooter.spinUp(ShooterIO.SHOOTING_OUTPUT, ShooterIO.SHOOTING_OUTPUT)
            .andThen(telescope.extend(TelescopeIO.SHOOTING))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.pivot(ShooterIO.SUBWOOFER_TILT));
    }

    public Command lowFerry() {
        return shooter.pivot(ShooterIO.SUBWOOFER_TILT)
            .andThen(telescope.extend(TelescopeIO.SHOOTING))
            .andThen(shooter.spinUp(0.9, 0.9));
    }

    public Command handoff() {
        return intake.run(IntakeIO.PICKUP_SPEED)
            .andThen(shooter.pivot(ShooterIO.HANDOFF_TILT))
            .andThen(telescope.extend(TelescopeIO.HANDOFF))
            .andThen(intake.zero())
            .andThen(shooter.load(ShooterIO.LOADER_HANDOFF_OUTPUT))
            .andThen(intake.run(IntakeIO.EJECT_SPEED))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(backOut())
            .andThen(shooter.pivot(ShooterIO.STOW_TILT))
            .andThen(intake.run(IntakeIO.OFF_SPEED));
    }

    public Command startIntake() {
        return shooter.pivot(ShooterIO.HANDOFF_TILT)
            .andThen(Commands.waitSeconds(0.3))
            .andThen(intake.tilt(IntakeIO.GROUND_TILT)
            .andThen(intake.run(IntakeIO.PICKUP_SPEED))
        );
    }

    public Command ejectLoader() {
        return shooter.pivot(ShooterIO.HANDOFF_TILT)
            .andThen(Commands.waitSeconds(0.25))
            .andThen(shooter.load(1))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.load(ShooterIO.LOADER_OFF_OUTPUT))
            .andThen(stow());
    }

    public Command shoot() {
        return shooter.load(ShooterIO.LOADER_SHOOTING_OUTPUT)
            .andThen(Commands.waitSeconds(0.9))
            .andThen(shooter.load(ShooterIO.LOADER_OFF_OUTPUT))
            .andThen(stow());

    }

    public Command stow() {
        return intake.zero()
            .andThen(intake.run(IntakeIO.OFF_SPEED))
            .andThen(telescope.coast())
            .andThen(climber.extend(ClimberIO.ZERO))
            .andThen(shooter.spinUp(ShooterIO.OFF_OUTPUT, ShooterIO.OFF_OUTPUT))
            .andThen(shooter.load(ShooterIO.LOADER_OFF_OUTPUT))
            .andThen(Commands.waitSeconds(0.25))
            .andThen(shooter.pivot(ShooterIO.STOW_TILT));
    }

    public Command readyVariableShot() {
        return shooter.spinUp(ShooterIO.SHOOTING_OUTPUT, ShooterIO.SHOOTING_OUTPUT)
            .andThen(telescope.extend(TelescopeIO.SHOOTING))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(shooter.autoPivot());
    }

    public Command backOut() {
        return shooter.load(0.25)
            .andThen(shooter.spinUp(-0.15, -0.15 ))
            .andThen(Commands.waitSeconds(0.125))
            .andThen(shooter.load(0))
            .andThen(shooter.spinUp(0, 0));
    }

    public Command climberUp() {
        return shooter.pivot(ShooterIO.AMP_TILT)
            .andThen(intake.tilt(IntakeIO.GROUND_TILT))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(climber.extend(ClimberIO.EXTENDED));
    }

    public Command climberDown() {
        return climber.extend(ClimberIO.RETRACTED)
            .andThen(Commands.waitSeconds(2))
            .andThen(shooter.pivot(ShooterIO.STOW_TILT));
    }
}