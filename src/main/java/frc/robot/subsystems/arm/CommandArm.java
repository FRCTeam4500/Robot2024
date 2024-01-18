package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ExtendedMath;

public class CommandArm extends SubsystemBase { //runs commands

    public CommandArm(){
        super();
    }
    public static

    public Command wait(){
        return Commands.waitUntil(() -> atTargetState());
    }
    public boolean atTargetState() { //returns your target state
        return ExtendedMath.within(armMotor.getAngle(), ArmState.position, );
    }
}
