package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import static frc.robot.CANConstants.*;
import static com.ctre.phoenix6.signals.InvertedValue.*;
import static com.revrobotics.CANSparkLowLevel.MotorType.*;

public class SwerveConstants {
    /** Drive rotations per motor rotation */
    public static final double DRIVE_RATIO = 1/7.5;
    /** Angle rotations per motor rotation */
    public static final double ANGLE_RATIO = 1/6.75;

    public static final double MAX_LINEAR_SPEED_MPS = 5.0;
    public static final double WHEEL_DIAMETER_METERS = 0.1016;

    /* Sensitivities */
    public static final double MAX_FORWARD_SENSITIVITY = 5;
    public static final double MAX_SIDEWAYS_SENSITIVITY = 5;
    public static final double MAX_ROTATIONAL_SENSITIVITY = 3.5;
    public static final double MIN_SENSITIVITY = 0.2;

    /* Drive Modes */
    public static enum DriveMode {
        AngleCentric,
        RobotCentric,
        AlignToTarget,
        FieldCentric,
        XanderDrive
    }
    public static final TalonFXConfiguration driveConfig =
        new TalonFXConfiguration()
            .withSlot1(new Slot1Configs()
                .withKP(0.11)
                .withKI(0.5)
                .withKD(0.0001)
                .withKV(0.12))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(25)
                .withSupplyCurrentLimitEnable(true)
            );


    public static final SwerveMotor FRONT_LEFT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_FRONT_LEFT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(Clockwise_Positive))
        );
    public static final SwerveMotor FRONT_LEFT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_FRONT_LEFT_ANGLE_ID, kBrushless),
            motor -> {}
        );
    public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
        0.2974,
        0.2974
    );

    public static final SwerveMotor FRONT_RIGHT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_FRONT_RIGHT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(CounterClockwise_Positive))
        );
    public static final SwerveMotor FRONT_RIGHT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_FRONT_RIGHT_ANGLE_ID, kBrushless),
            motor -> {}
        );
    public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
        0.2974,
        -0.2974
    );

    public static final SwerveMotor BACK_LEFT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_BACK_LEFT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(Clockwise_Positive))
        );
    public static final SwerveMotor BACK_LEFT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_BACK_LEFT_ANGLE_ID, kBrushless),
            motor -> {}
        );
    public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
        -0.2974,
        0.2974
    );

    public static final SwerveMotor BACK_RIGHT_DRIVE_MOTOR =
        SwerveMotor.fromTalonFX(
            new TalonFX(SWERVE_BACK_RIGHT_DRIVE_ID),
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(CounterClockwise_Positive))
        );
    public static final SwerveMotor BACK_RIGHT_ANGLE_MOTOR =
        SwerveMotor.fromSparkMax(
            new CANSparkMax(SWERVE_BACK_RIGHT_ANGLE_ID, kBrushless),
            motor -> {}
        );
    public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
        -0.2974,
        -0.2974
    );
}