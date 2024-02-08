package frc.robot.subsystems.shooter2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.subsystems.shooter2.Shooter2Constants.*;

public class CommandShooter2 extends Shooter2 {
    private static CommandShooter2 instance;
    public static synchronized CommandShooter2 getInstance() {
        if (instance == null) instance = new CommandShooter2();
        return instance;
    }

    private CommandShooter2() {
        super();
    }

    public Command readyHandoff() {
        return Commands.runOnce(
            () -> pivot(HANDOFF_ANGLE), this
        ).andThen(
            Commands.waitSeconds(1)
        );
    }

    public Command runHandoff() {
        return Commands.runOnce(
            () -> load()
        ).andThen(
            Commands.waitSeconds(0.5)
        ).andThen(
            Commands.runOnce(() -> stopLoading())
        ).andThen(
            Commands.runOnce(() -> pivot(STOW_ANGLE))
        );
    }

    public Command shoot() {
        return Commands.runOnce(() -> load());
    }

    public Command readySpeaker() {
        return Commands.runOnce(
            () -> {
                shoot(1);
                pivot(calculatePivot());
            }
        );
    }

    public Command readyAmp() {
        return Commands.runOnce(
            () -> {
                shoot(AMP_OUTPUT);
                pivot(AMP_ANGLE);
            }
        );
    }

    public Command zero() {
        return Commands.runOnce(
            () -> {
                stopLoading();
                shoot(0);
                pivot(STOW_ANGLE);
            }
        );
    }
}
