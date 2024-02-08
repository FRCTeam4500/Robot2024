package frc.robot.subsystems.shooter3;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.subsystems.shooter3.Shooter3Constants.*;

public class CommandShooter3 extends Shooter3 {
    private static CommandShooter3 instance;
    public static synchronized CommandShooter3 getInstance() {
        if (instance == null) instance = new CommandShooter3();
        return instance;
    }

    public CommandShooter3() {
        super();
    }
    
    public Command readyHandoffCommand() {
        return Commands.runOnce(
            () -> setPivotAngle(READY_HANDOFF_ANGLE), this
        );
    }

    public Command executeHandoffCommand() {
        return Commands.runOnce(
            () -> setPivotAngle(HANDOFF_ANGLE), this
        ).andThen(
            Commands.waitSeconds(1)
        ).andThen(
            load()
        ).andThen(
            Commands.runOnce(() -> setPivotAngle(READY_HANDOFF_ANGLE))
        );
    }

    public Command zero() {
        return Commands.runOnce(
            () -> {
                setPivotAngle(STOW_ANGLE);
                setLoaderOutput(0);
                setShooterOutput(0, 0);
            }, this
        );
    }
    
    public Command load() {
        return Commands.runOnce(
            () -> setLoaderOutput(LOADING_OUTPUT), this
        ).andThen(
            Commands.waitSeconds(1.5)
        ).until(
            this::seesNote
        ).andThen(
            Commands.runOnce(() -> setLoaderOutput(0))
        );
    }

    public Command readyAmp() {
        return Commands.runOnce(
            () -> {
                setPivotAngle(AMP_ANGLE);
                setShooterOutput(SHOOT_OUTPUT_LEFT, SHOOT_OUTPUT_RIGHT);
            }
        );
    }
    
    public Command readySpeaker(double distance){
        return Commands.runOnce(
            () -> {
                setPivotAngle(calcSpeakerAngle(distance));
                setShooterOutput(SHOOT_OUTPUT_LEFT, SHOOT_OUTPUT_RIGHT);
            }
        );
    }

    public Command shoot() {
        return Commands.runOnce(
            () -> setLoaderOutput(LOADING_OUTPUT), this
        ).andThen(Commands.waitSeconds(0.5)
        ).andThen(Commands.runOnce(() -> setShooterOutput(0, 0)));
    }
}