package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
		setupAuto();
		setupDriveController();
		setupOperatorController();
	}

	private void setupAuto() {
		autonChooser = AutoBuilder.buildAutoChooser();
		Shuffleboard.getTab("Display").add(autonChooser);
	}

	private void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		structure.setDefaultDrive(xbox);
		
		Trigger cancelAllButton = xbox.start();
		Trigger resetGyroButton = xbox.a();
		Trigger alignAmpButton = xbox.rightTrigger();
		Trigger alignAmpShotButton = xbox.povDown();
		Trigger alignFarShotButton = xbox.povUp();
		Trigger faceSpeakerButton = xbox.b();

		cancelAllButton.onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
		resetGyroButton.onTrue(structure.resetGyro());
		alignAmpButton.whileTrue(structure.driveToAmp());
		alignAmpShotButton.whileTrue(structure.driveToAmpShot());
		alignFarShotButton.whileTrue(structure.driveToFarShot());
		faceSpeakerButton.whileTrue(structure.alignToSpeaker(xbox));
	}

	private void setupOperatorController() {
		flightSim = new CommandJoystick(OPERATOR_PORT);
		
		Trigger shootButton = flightSim.button(1).debounce(0.1, DebounceType.kBoth);
		Trigger intakeButton = flightSim.button(2).debounce(0.1, DebounceType.kBoth);
		Trigger readyEverywhereButton = flightSim.button(3).debounce(0.1, DebounceType.kBoth);
		Trigger readySubwooferButton = flightSim.button(4).debounce(0.1, DebounceType.kBoth);
		Trigger readyAmpButton = flightSim.button(5).debounce(0.1, DebounceType.kBoth);
		// 6
		Trigger climberDownButton = flightSim.button(7).debounce(0.1, DebounceType.kBoth);
		Trigger climberUpButton = flightSim.button(8).debounce(0.1, DebounceType.kBoth);
		Trigger lowFerryButton = flightSim.button(9).debounce(0.1, DebounceType.kBoth);
		Trigger stowButton = flightSim.button(10);
		Trigger ejectButton = flightSim.button(11).debounce(0.1, DebounceType.kBoth);
		Trigger unstickShooterButton = flightSim.button(12).debounce(0.1, DebounceType.kBoth);
		Trigger zeroIntakeButton = flightSim.povDown().debounce(0.1, DebounceType.kBoth);

		shootButton.onTrue(structure.shoot());
		intakeButton.onTrue(structure.startIntake());
		intakeButton.onFalse(structure.handoff());
		readyEverywhereButton.onTrue(structure.readyVariableShot());
		readySubwooferButton.onTrue(structure.readySubwooferShot());
		readyAmpButton.onTrue(structure.readyAmp());
		climberDownButton.onTrue(structure.climberDown());
		climberUpButton.onTrue(structure.climberUp());
		lowFerryButton.onTrue(structure.lowFerry());
		stowButton.onTrue(structure.stow());
		ejectButton.onTrue(structure.ejectLoader());
		unstickShooterButton.onTrue(structure.backOut());
		zeroIntakeButton.whileTrue(structure.zeroIntake());
		zeroIntakeButton.onFalse(structure.stow());
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
		if (autoCommand != null) autoCommand.cancel();
	}
}