package frc.robot.subsystems.shooter;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static frc.robot.CANConstants.LOADER_ID;
import static frc.robot.CANConstants.SHOOTER_ONE_ID;
import static frc.robot.CANConstants.SHOOTER_TWO_ID;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotorController;
// q

import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase implements LoggableInputs {
    
    private static Shooter instance;
    // hi
    public static synchronized Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    private SparkMaxMotorController shootshootMotorOne;
    private SparkMaxMotorController shootshootMotorTwo;
    private SparkMaxMotorController loaderMotor;
    private ShooterState state;
    protected Shooter() {
        shootshootMotorOne = new SparkMaxMotorController(SHOOTER_ONE_ID, MotorType.kBrushless);
        shootshootMotorTwo = new SparkMaxMotorController(SHOOTER_TWO_ID, MotorType.kBrushless);
        loaderMotor = new SparkMaxMotorController(LOADER_ID, MotorType.kBrushless);
        state = ShooterState.Off;

        
    }

    public void setState(ShooterState state) {
        shootshootMotorOne.setOutput(state.shooterSpeed);
        shootshootMotorTwo.setOutput(state.shooterSpeed);
        loaderMotor.setOutput(state.loaderSpeed);
    }

    public boolean spunUp() {
        return Math.abs(shootshootMotorOne.getOutput() - shooterShooting) < threshold &&
        Math.abs(shootshootMotorTwo.getOutput() - shooterShooting) < threshold;
    }

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toLog'");
    }

    @Override
    public void fromLog(LogTable table) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
    }
}
