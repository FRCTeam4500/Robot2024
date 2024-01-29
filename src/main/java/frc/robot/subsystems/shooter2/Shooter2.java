package frc.robot.subsystems.shooter2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;

import static frc.robot.CANConstants.*;
import static frc.robot.subsystems.shooter2.Shooter2Constants.*;

public class Shooter2 extends SubsystemBase {
    private static Shooter2 instance;
    public static synchronized Shooter2 getInstance() {
        if (instance == null) instance = new Shooter2();
        return instance;
    }

    private SparkMaxMotor leftShooter;
    private SparkMaxMotor rightShooter;
    private SparkMaxMotor loader;
    private SparkMaxMotor pivoter;
    protected Shooter2() {
        leftShooter = new SparkMaxMotor(SHOOTER_ONE_ID);
        rightShooter = new SparkMaxMotor(SHOOTER_TWO_ID);
        loader = new SparkMaxMotor(LOADER_ID);
        pivoter = new SparkMaxMotor(SHOOTER_PIVOT_ID);
    }

    public void shoot(double output) {
        leftShooter.set(output);
        rightShooter.set(-output);
    }

    public void load() {
        loader.set(LOADING_OUTPUT);
    }

    public void pivot(Rotation2d angle) {
        pivoter.setAngle(angle);
    }

    public static Rotation2d calcAngle(double distanceMeters) {
        return new Rotation2d();
    }
}
