package frc.robot.subsystems.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANConstants;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase implements LoggableInputs {
    
    private static Climber instance;
    public static synchronized Climber getInstance() //synchronized is a safety measure
    {
        if(instance == null)
        {
            instance = new Climber();
        }
        return instance;
    }

    private CANSparkMax climbMotor;
    public static final double ZERO = 0.0;
    public static final double EXTENDED = -130;
    public static final double RETRACTED = 30;

    private Climber()
    {
        climbMotor = new CANSparkMax(CANConstants.CLIMBER_ID, MotorType.kBrushless);
        climbMotor.setIdleMode(IdleMode.kBrake);
        climbMotor.getPIDController().setP(5);
        climbMotor.getPIDController().setOutputRange(-1, 1);

    }

    public Command extend(double extension)
    {
        return Commands.runOnce(()->{//this is a lamda
            climbMotor.getPIDController().setReference(extension, ControlType.kPosition);
        }, this);//this refers to a Climber object we weill make in the future

    }    

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Climb Extension", ()->{
            return climbMotor.getEncoder().getPosition();
        }, null);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Extension", climbMotor.getEncoder().getPosition());
    }

    @Override
    public void fromLog(LogTable table) {}
}
