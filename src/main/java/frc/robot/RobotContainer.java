package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
	private CommandXboxController xbox;
	private CommandJoystick flightSim;
    private Superstructure structure;
	private Command autoCommand;
	private SendableChooser<Command> autonChooser;

	private final int DRIVER_PORT = 2;
	private final int OPERATOR_PORT = 1;

	public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
		structure = Superstructure.getInstance();
		setupDriveController();
		setupOperatorController();
		// structure.debugToShuffleboard();
	}

	private void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		structure.setDefaultDrive(xbox);

		Trigger resetGyroButton = xbox.a();
		Trigger alignSpeaker = xbox.rightTrigger();
		resetGyroButton.onTrue(structure.resetGyro());
		alignSpeaker.whileTrue(structure.alignToSpeaker(xbox));
	}

	private void setupOperatorController() {
		flightSim = new CommandJoystick(OPERATOR_PORT);
		Trigger intakeButton = flightSim.button(2);
		Trigger shootButton = flightSim.button(1);
		Trigger stowButton = flightSim.button(10);
		Trigger ampButton = flightSim.button(5);
		Trigger subwooferButton = flightSim.button(6);

		intakeButton.onTrue(structure.groundIntake());
		intakeButton.onFalse(structure.handoff());
		shootButton.onTrue(structure.shoot());
		stowButton.onTrue(structure.stow());
		ampButton.onTrue(structure.readyAmp());
		subwooferButton.onTrue(structure.readySubwoofer());
	}

	public Command rumbleCommand(double timeSeconds) {
		return Commands.startEnd(
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0.5),
			() -> xbox.getHID().setRumble(RumbleType.kBothRumble, 0)
		).withTimeout(timeSeconds);
	}

	public void autonomousInit() {
		autoCommand = autonChooser.getSelected();
		if (autoCommand != null) autoCommand.schedule();

	}

	public void teleopInit() {
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}