package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.telescope.TelescopeIO;
import frc.robot.subsystems.vision.AprilTagVision;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

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
		Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
		Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		LoggedPowerDistribution.getInstance(1, ModuleType.kRev); // Enables power distribution logging
		Logger.start();
	}

	private void startLogging() {
		new Timer().schedule(
			new TimerTask() {
				public void run() {
					Logger.processInputs("Swerve", Swerve.getInstance());
					Logger.processInputs("Tag Vision", AprilTagVision.getInstance());
					Logger.processInputs("Intake", IntakeIO.getInstance());
					Logger.processInputs("Shooter", ShooterIO.getInstance());
					Logger.processInputs("Climber", ClimberIO.getInstance());
					Logger.processInputs("Telescope", TelescopeIO.getInstance());
				}
			},
			10,
			20
		);
	}
}
