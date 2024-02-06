package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.CommandIntake;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
	private RobotContainer robotContainer;

	@Override
	public void robotInit() {
		initLogging();
		robotContainer = new RobotContainer();
		startLogging();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		Logger.processInputs("Swerve", SwerveDrive.getInstance());
		Logger.processInputs("Tag Vision", AprilTagVision.getInstance());
		Logger.processInputs("Piece Vision", GamePieceVision.getInstance());
		Logger.processInputs("Intake", CommandIntake.getInstance());
	}

	@Override
	public void autonomousInit() {
		robotContainer.autonomousInit();
	}

	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}

	private void initLogging() {
		Logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);
		switch (BuildInfo.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}
		// Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
		Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
		Logger.start();
	}

	private void startLogging() {
		new Timer().schedule(
			new TimerTask() {
				public void run() {
					// TODO: Change these!
					Logger.processInputs("Swerve", SwerveDrive.getInstance());
					Logger.processInputs("Tag Vision", AprilTagVision.getInstance());
					Logger.processInputs("Piece Vision", GamePieceVision.getInstance());
					Logger.processInputs("Intake", Intake.getInstance());
				}
			}, 
			10, 
			20
		);
	}
}
