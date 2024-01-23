package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmConstants.ArmState;

/**
 * @author Max
 * @author Yijia
 * @author Oliver
 */
public class CommandArm extends Arm { // runs commands

    public CommandArm(){
        super();
    }

    public Command waitUntilAtTargetState(){
        return Commands.waitUntil(() -> atTargetState());
    }

    public Command goToZeroCommand()
    {
        return Commands.runOnce(
            () -> setState(ArmState.ZERO), this
        ).andThen(
            waitUntilAtTargetState()
        );
    }

    /**
     * sets arm to shoot in speaker
     * @author Bennett
     */
    public Command goToSpeakerCommand()
    {
        return Commands.runOnce(
            () -> setState(ArmState.SPEAKER), this
        ).andThen(
            waitUntilAtTargetState()
        );
    }

     /**
     * sets arm to shoot in amp
     * @author Bennett
     */
    public Command goToAmpCommand()
    {
        return Commands.runOnce(
            () -> setState(ArmState.AMP), this
        ).andThen(
            waitUntilAtTargetState()
        );
    }

     /**
     * sets arm for handoff
     * @author Bennett
     */
    public Command goToHandoffCommand()
    {
        return Commands.runOnce(
            () -> setState(ArmState.HANDOFF), this
        ).andThen(
            waitUntilAtTargetState()
        );
    }
  
}
