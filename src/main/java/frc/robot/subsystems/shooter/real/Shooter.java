package frc.robot.subsystems.shooter.real;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.telescope.TelescopeIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.CANConstants.*;

import org.littletonrobotics.junction.LogTable;

public class Shooter extends ShooterIO {
    public static final double AMP_TILT = -16.3;//-18;
    public static final double HANDOFF_TILT = -6;//-7;
    public static final double SUBWOOFER_TILT = 1.5;//1;
    public static final double STAGE_TILT = -2.2;
    public static final double STOW_TILT = 0;//-6.15;
    public static final double SUBWOOFER_LEFT_SPEED =  1;
    public static final double SUBWOOFER_RIGHT_SPEED = 1;
    public static final double AMP_SPEED = 0.5;
    public static final double OFF_SPEED = 0;
    public static final double LOADER_HANDOFF_SPEED = -0.75;
    public static final double LOADER_SHOOT_SPEED = -1;
    public static final double LOADER_OFF_SPEED = 0;
    
    private CANSparkMax tiltMotor;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax loaderMotor;
    private InterpolatingDoubleTreeMap angleCalculator;
    private MechanismLigament2d shooterState;
    public Shooter() {
        rightMotor = new CANSparkMax(SHOOTER_ONE_ID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(SHOOTER_TWO_ID, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(SHOOTER_PIVOT_ID, MotorType.kBrushless);
        loaderMotor = new CANSparkMax(LOADER_ID, MotorType.kBrushless);

        rightMotor.getPIDController().setP(0.3);
        tiltMotor.getPIDController().setP(3);
        tiltMotor.getPIDController().setOutputRange(-0.5, 0.5);
        tiltMotor.setIdleMode(IdleMode.kCoast);

        rightMotor.setSmartCurrentLimit(30);
        leftMotor.setSmartCurrentLimit(30);
        tiltMotor.setSmartCurrentLimit(30);
        loaderMotor.setSmartCurrentLimit(40);

        angleCalculator = new InterpolatingDoubleTreeMap();
        angleCalculator.put(1.37, 1.25);
        angleCalculator.put(1.74, -0.25);
        angleCalculator.put(2.14, -2.25);
        angleCalculator.put(2.8, -3.25);
        angleCalculator.put(3.25, -3.75);
        angleCalculator.put(3.75, -4.25);
        angleCalculator.put(4.4, -4.6);

        shooterState = new MechanismLigament2d("Shooter State", 0.3, 140);
        TelescopeIO.getInstance().getCurrentMech().append(shooterState);
    }

    public Command pivot(double angle) {
        return Commands.runOnce(
            () -> tiltMotor.getPIDController().setReference(angle, ControlType.kPosition, 0, calcFF(angle)),
            this
        );
    }

    public Command autoPivot() {
        return Commands.run(
            () -> {
                double distance = SwerveIO.getInstance().getEstimatedPose().getTranslation().getDistance(new Translation2d(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 16, 5.9
                ));
                double angle = angleCalculator.get(distance);
                tiltMotor.getPIDController().setReference(angle, ControlType.kPosition, 0, calcFF(angle));
            }, this
        );
    }

    public Command load(double output) {
        return Commands.runOnce(
            () -> loaderMotor.set(output), this
        );
    }

    public Command spinUp(double left, double right) {
        return Commands.runOnce(
            () -> {
                leftMotor.set(left);
                rightMotor.set(right);
            }, this
        );
    }

    public Command coast() {
        return Commands.runOnce(() -> tiltMotor.set(0), this);
    }

    public double calcFF(double setpoint) {
        double angle =  -(setpoint * 360 / 69.9 + 38);
        double kG = -0.3516;
        return Math.cos(Math.toRadians(angle) * kG);
      }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Left Speed", () -> leftMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Right Speed", () -> rightMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Loader Speed", () -> loaderMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("Tilt", () -> tiltMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Actual Angle?", () -> 38 + tiltMotor.getEncoder().getPosition() * 360 / 69.9, null);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Left Speed", leftMotor.getEncoder().getVelocity());
        table.put("Right Speed", rightMotor.getEncoder().getVelocity());
        table.put("Loader Speed", loaderMotor.getEncoder().getVelocity());
        table.put("Tilt", tiltMotor.getEncoder().getPosition());
        shooterState.setAngle(140 + (4.5 * tiltMotor.getEncoder().getPosition()));
    }

    @Override
    public void fromLog(LogTable table) {}
}
