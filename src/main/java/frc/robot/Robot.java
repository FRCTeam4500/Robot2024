package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.messaging.Messaging;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.GamePieceVision;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private RobotContainer robotContainer;
	private Timer timer;

	@Override
	public void robotInit() {
		timer = new Timer();

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
		robotContainer = new RobotContainer();
		timer.schedule(
			new TimerTask() {
				public void run() {
					// TODO: Change these!
					Logger.processInputs("Swerve", SwerveDrive.getInstance());
					Logger.processInputs("Tag Vision", AprilTagVision.getInstance());
					Logger.processInputs("Piece Vision", GamePieceVision.getInstance());
					Logger.processInputs("Messaging", Messaging.getInstance());
				}
			}, 
			10, 
			20
		);
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
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
