package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ExtendedMath;

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
    
}
