package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

/**
 * @author Max
 * @author Yijia
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
  
}
