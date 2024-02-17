package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

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
		setupAuto();
		setupDriveController();
		setupOperatorController();
		// structure.debugToShuffleboard();
	}

	private void setupAuto() {
		autonChooser = AutoBuilder.buildAutoChooser();
	}

	private void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		structure.setDefaultDrive(xbox);

		Trigger resetGyroButton = xbox.a();
		Trigger alignAmp = xbox.rightTrigger();
		resetGyroButton.onTrue(structure.resetGyro());
		alignAmp.whileTrue(structure.driveToAmp());
	}

	private void setupOperatorController() {
		flightSim = new CommandJoystick(OPERATOR_PORT);
		Trigger intakeButton = flightSim.button(2);
		Trigger shootButton = flightSim.button(1);
		Trigger stowButton = flightSim.button(10);
		Trigger readyAmpButton = flightSim.button(5);
		Trigger resetIntakeButton = flightSim.button(12);
		Trigger confirmIntakeButton = flightSim.button(4);

		intakeButton.onTrue(structure.groundIntake());
		intakeButton.onFalse(structure.stow());
		shootButton.onTrue(structure.shootWithEverything());
		shootButton.onFalse(structure.stow());
		stowButton.onTrue(structure.stow());
		readyAmpButton.onTrue(structure.handoffAndAmp());
		readyAmpButton.onFalse(structure.shoot().andThen(structure.stow()));
		resetIntakeButton.onTrue(structure.resetIntake());
		confirmIntakeButton.onTrue(structure.confirmIntake());
		confirmIntakeButton.onFalse(structure.offIntake());
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