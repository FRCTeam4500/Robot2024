package frc.robot.subsystems.arm2;

public class CommandArm2 {
    private static CommandArm2 instance;
    public static synchronized CommandArm2 getInstance() {
        if (instance == null) instance = new CommandArm2();
        return instance;
    }


}//poop