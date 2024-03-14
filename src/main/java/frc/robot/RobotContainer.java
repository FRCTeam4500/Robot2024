package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

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
		Trigger intakeButton = flightSim.button(2);
		Trigger subwooferShotButton = flightSim.button(1);
		Trigger stowButton = flightSim.button(10);
		Trigger ampButton = flightSim.button(5);
		Trigger ejectButton = flightSim.button(11);
		Trigger handoffButton = flightSim.button(6);
		Trigger variableShotButton = flightSim.button(3);
		Trigger unstickShooterButton = flightSim.button(12);
		Trigger climberUpButton = flightSim.button(8);
		Trigger climberDownButton = flightSim.button(7);
		Trigger zeroIntakeButton = flightSim.povDown();

		climberUpButton.onTrue(structure.climberUp());
		climberDownButton.onTrue(structure.climberDown());
		intakeButton.onTrue(structure.startIntake());
		intakeButton.onFalse(structure.handoff());
		subwooferShotButton.onTrue(structure.readySubwooferShot());
		subwooferShotButton.onFalse(structure.shoot());
		variableShotButton.onTrue(structure.readyVariableShot());
		variableShotButton.onFalse(structure.shoot());
		ampButton.onTrue(structure.readyAmp());
		ampButton.onFalse(structure.shoot());
		ejectButton.onTrue(structure.ejectLoader());
		ejectButton.onFalse(structure.stow());
		stowButton.onTrue(structure.stow());
		handoffButton.onTrue(structure.handoff());
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
		if (autoCommand != null) {
			autoCommand.cancel();
		}
	}
}