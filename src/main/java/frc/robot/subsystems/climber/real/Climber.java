package frc.robot.subsystems.climber.real;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CANConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberIO;

import org.littletonrobotics.junction.LogTable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends ClimberIO {
    private CANSparkMax climbMotor;
    private Mechanism2d currentMech;
    private MechanismRoot2d root;
    private MechanismLigament2d climberMech;

    public Climber()
    {
        climbMotor = new CANSparkMax(CANConstants.CLIMBER_ID, MotorType.kBrushless);
        climbMotor.setIdleMode(IdleMode.kBrake);
        climbMotor.getPIDController().setP(5);
        climbMotor.getPIDController().setOutputRange(-1, 1);
        currentMech = Superstructure.getCurrentMech();
        root = currentMech.getRoot("Climber Root", 0.6858, 0.1);
        climberMech = new MechanismLigament2d("Climber State", 0.25, 90);
        root.append(climberMech);
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
        climberMech.setLength(-climbMotor.getEncoder().getPosition() * 0.004 + 0.25);
    }

    @Override
    public void fromLog(LogTable table) {}
}
