package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

/**
 * A wraqqer class around {@link Intake Intake} which provides command functionality.
 * @author Billy Bonga
 */
public class CommandIntake extends Intake {
    // Singleton stuff
    private static CommandIntake instance;
    public static synchronized CommandIntake getInstance() {
        if (instance == null) instance = new CommandIntake();
        return instance;
    }

    /**
     * The constructor, which is run on object creation. Since this class doesn't
     * hold any state, we just call our super constructor, which calls
     * {@link Intake Intake's} constructor
     * @author Vimal Buckley
     */
    private CommandIntake() {
        super();
    }

    /**
     * A command that waits until the intake has reached its target tilt
     * @author Vimal Buckley
     */
    public Command waitUntilAtTarget() {
        // Quick lamda (() -> {}) syntax intro
        // Stuff in the () is the arguments of the function
        // Stuff in the {} is the code of the function
        // If there is exactly one argument, () can be ommited
        // If there is exactly one line of code, and it returns a value,
        // the {} can be ommited, along with the return keyword
        // The below lamda could also be written as:
        //                        () -> {return atTargetState;}
        return Commands.waitUntil(() -> atTargetState());
    }

    /**
     * A command that stows (zeroes) the intake
     * @author Vimal Buckley
     */
    public Command stow() {
        return Commands.runOnce(
            () -> setState(IntakeState.Stow), this
        ).andThen(
            waitUntilAtTarget()
        );
    }

    /**
     * A command that handoffs a piece to the loader/shooter
     * @author Vimal Buckley
    */
    public Command handoff() {
        return Commands.runOnce(
            () -> setState(IntakeState.ReadyHandoff), this
        ).andThen(
            waitUntilAtTarget()
        ).andThen(
            () -> setState(IntakeState.ExecuteHandoff), this
        );
    }

    /**
     * A command that pickups a game piece, then stows the intake
     * @author Bimal Vuckley
    */
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