package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
		setupAuto();
		setupDriveController();
		setupOperatorController();
		structure.debugToShuffleboard();
	}

	private void setupAuto() {
		autonChooser = AutoBuilder.buildAutoChooser();
		Shuffleboard.getTab("Display").add(
			"Auto Route", 
			autonChooser
		);
	}

	private void setupDriveController() {
		xbox = new CommandXboxController(DRIVER_PORT);
		structure.setDefaultDrive(xbox);
		
		Trigger resetGyroButton = xbox.a();

		resetGyroButton.onTrue(structure.resetGyro());
	}

	private void setupOperatorController() {
		flightSim = new CommandJoystick(OPERATOR_PORT);
		Trigger intakeButton = flightSim.button(2);
		Trigger outtakeButton = flightSim.button(1);

		intakeButton.onTrue(structure.runIntake(0.4));
		intakeButton.onFalse(structure.runIntake(0));
		outtakeButton.onTrue(structure.runIntake(-0.4));
		outtakeButton.onFalse(structure.runIntake(0));
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