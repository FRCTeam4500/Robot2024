package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

public class CommandIntake extends Intake {
    private static CommandIntake instance;
    public static synchronized CommandIntake getInstance() {
        if (instance == null) instance = new CommandIntake();
        return instance;
    }

    private CommandIntake() {
        super();
    }

    public Command waitUntilAtTarget() {
        return Commands.waitUntil(() -> atTargetState());
    }

    public Command stow() {
        return Commands.runOnce(
            () -> setState(IntakeState.Stow), this
        ).andThen(
            waitUntilAtTarget()
        );
    }

    public Command handoff() {
        return Commands.runOnce(
            () -> setState(IntakeState.ReadyHandoff), this
        ).andThen(
            waitUntilAtTarget()
        ).andThen(
            () -> setState(IntakeState.ExecuteHandoff), this
        ).andThen(
            waitUntilAtTarget()
        );
    }

    public Command pickup() {
        return Commands.runOnce(
            () -> setState(IntakeState.ReadyPickup), this
        ).andThen(
            waitUntilAtTarget()
        ).andThen(
            () -> setState(IntakeState.ExecutePickup), this
        ).andThen(
            Commands.waitUntil(() -> hasNote())
        ).andThen(
            stow()
        ).andThen(
            waitUntilAtTarget()
        );
    }
}
