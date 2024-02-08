package frc.robot.subsystems.shooter3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.hardware.SparkMaxMotor;

import static frc.robot.CANConstants.*;


public class Shooter3 extends SubsystemBase{
    private static Shooter3 instance;
    public static synchronized Shooter3 getInstance() {
        if (instance == null) instance = new Shooter3();
        return instance;
    }      

    private SparkMaxMotor leftMotor;
    private SparkMaxMotor rightMotor;
    private SparkMaxMotor loaderMotor;
    private SparkMaxMotor pivotMotor;

    private AnalogInput gamePieceLidar;

    protected Shooter3() {
        leftMotor = new SparkMaxMotor(SHOOTER_ONE_ID);
        rightMotor = new SparkMaxMotor(SHOOTER_TWO_ID);
        pivotMotor = new SparkMaxMotor(SHOOTER_PIVOT_ID);
        loaderMotor = new SparkMaxMotor(LOADER_ID);
        gamePieceLidar = new AnalogInput(GAMEPIECE_LIDAR_CHANNEL);
    }
    
    public void setShooterOutput(double left, double right) {
        leftMotor.setOutput(left);
        rightMotor.setOutput(right);
    }

    public void setLoaderOutput(double output) {
        loaderMotor.setOutput(output);
    }

    public void setPivotAngle(Rotation2d angle) {
        pivotMotor.setAngle(angle);
    }

    public boolean seesNote() {
        return gamePieceLidar.getValue() < 2200;
    }

    public static Rotation2d calcSpeakerAngle(double distance) {
        return new Rotation2d(distance, 1.45);
    }
}
