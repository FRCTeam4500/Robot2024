package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;

/**
 * @author Gasya
 * @author Ashwin
 */
public class CommandClimber extends Climber {
    private static CommandClimber instance;
    public static synchronized CommandClimber getInstance(){
        if (instance == null) instance = new CommandClimber();
        return instance;

    }
    public Command climb() {
        return Commands.runOnce(
            () -> setState(ClimberState.Midhook)
        ).andThen(
        atClimberTargetState()
        );
    }
    public Command readyClimb() {
        return Commands.runOnce(
        () -> setState(ClimberState.Highhook)
        ).andThen(
            atClimberTargetState()
        );
    }
    public Command atClimberTargetState() {
        return Commands.waitUntil(() -> atTargetState());
    }
}
