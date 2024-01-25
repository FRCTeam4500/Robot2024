package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.messaging.Messaging;
import frc.robot.subsystems.swerve.CommandSwerve;

public class RobotContainer {
	private CommandXboxController xbox;
    private Superstructure structure;//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	private CommandSwerve swerve;//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	private Messaging messaging;//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	private Command autoCommand;//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	private SendableChooser<Command> autonChooser;//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	private final int DRIVER_PORT = 2;//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	public RobotContainer() {//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
        DriverStation.silenceJoystickConnectionWarning(true);//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
        structure = Superstructure.getInstance();//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
		swerve = CommandSwerve.getInstance();//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
		messaging = Messaging.getInstance();//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
		setupAuto();//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
		setupDriveController();//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP
	}//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAP//PPAPv

	public void setupAuto() {
		autonChooser = AutoBuilder.buildAutoChooser();
		Shuffleboard.getTab("Display").add(
			"Auto Route", 
			autonChooser
		);
	}

	public void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		swerve.setDefaultCommand(swerve.angleCentricDriveCommand(xbox));

		Trigger switchDriveModeButton = xbox.x();
		Trigger resetGyroButton = xbox.a();
		Trigger alignToTargetButton = xbox.rightBumper();
		Trigger cancelationButton = xbox.start();
		Trigger moveToAprilTagButton = xbox.leftBumper();

        moveToAprilTagButton.whileTrue(structure.driveToTag(new Pose2d(1, 0.25, new Rotation2d())));
        switchDriveModeButton.toggleOnTrue(structure.robotCentricDrive(xbox));
		resetGyroButton.onTrue(structure.resetGyro());
		alignToTargetButton.whileTrue(structure.driveToPiece());
		cancelationButton.onTrue(Commands.runOnce(
			() -> CommandScheduler.getInstance().cancelAll())
		);
	}

	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0)
		).withTimeout(timeSeconds);
	}

	public void autonomousInit() {
		messaging.setMessagingState(true);
		messaging.addMessage("Auto Started");
		autoCommand = autonChooser.getSelected();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			messaging.addMessage("No Auto Command Selected");
		}
	}

	public void teleopInit() {
		messaging.setMessagingState(true);
		messaging.addMessage("Teleop Started");
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}