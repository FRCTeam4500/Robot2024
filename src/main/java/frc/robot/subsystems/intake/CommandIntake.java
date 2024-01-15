package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSpeed;

public class CommandIntake extends Intake {
    private static CommandIntake instance;
    public static synchronized CommandIntake getInstance() {
        if (instance == null) instance = new CommandIntake();
        return instance;
    }

    private CommandIntake() {
        super();
    }

    /**
     * Set the current state of the intake to be zero.
     */
    public Command zero() {
        return Commands.runOnce(
            () -> setState(IntakePosition.Zero, IntakeSpeed.Off),
            this
        ).andThen(waitUntilAtTarget());
    }

    /**
     * Move the intake to the ground position and set the speed to be intaking.
     */
    public Command groundIntake() {
        return Commands.runOnce(
            () -> setState(IntakePosition.Ground, IntakeSpeed.Intaking),
            this
        )
            .andThen(waitUntilAtTarget())
            .andThen(Commands.waitUntil(() -> hasNote()))
            .andThen(zero());
    }

    /**
     * Move the intake to the handoff position 
     * and then handoff the note to the loader.
     */
    public Command handoff() {
        return Commands.runOnce(
            () -> setState(IntakePosition.Handoff, IntakeSpeed.Off), // tilt to handoff position
            this
        ).andThen(waitUntilAtTarget()) // When at handoff position
            .andThen(Commands.runOnce(() -> setState(IntakeSpeed.Ejecting))) // handoff
            .withTimeout(1) // for 1 second
            .andThen(zero()); // then go to zero
    }

    /**
     * Wait until the intake is at the target state.
     */
    public Command waitUntilAtTarget() {
        return Commands.waitUntil(
            () -> atPosition() && atSpeed()
        );
    }
}
