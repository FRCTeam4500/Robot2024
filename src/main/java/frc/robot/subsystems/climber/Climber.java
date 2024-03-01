package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;

public class Climber extends SubsystemBase {
    private static Climber instance;
    public static synchronized Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private CANSparkMax climbingMotor;
    public Climber() {
        climbingMotor = new CANSparkMax(CANConstants.CLIMBER_ID, MotorType.kBrushless);
        climbingMotor.getPIDController().setP(1);
    }

    public Command extend(double extension) {
        return Commands.runOnce(() -> climbingMotor.getPIDController().setReference(extension, ControlType.kPosition));
    }
}
