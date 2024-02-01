package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CANConstants;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.hardware.TalonFXMotor;

import static frc.robot.CANConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveConstants {
    /** Drive rotations per motor rotation
     * @param  poop
     */
    public static final double DRIVE_RATIO = 1/7.5;
    /** Angle rotations per motor rotation */
    public static final double ANGLE_RATIO = 1/6.75;

    public static final double MAX_LINEAR_SPEED_MPS = 4.;
    public static final double WHEEL_DIAMETER_METERS = 0.1016;

    /* Sensitivities */
    public static final double MAX_FORWARD_SENSITIVITY = 4;
    public static final double MAX_SIDEWAYS_SENSITIVITY = 4;
    public static final double MAX_ROTATIONAL_SENSITIVITY = 3.5;
    public static final double MIN_SENSITIVITY = 0.2;

    /* Drive Modes */
    public static enum DriveMode{
        AngleCentric,
        RobotCentric,
        AlignToTarget
    }
    public static final TalonFXConfiguration driveConfig = 
        new TalonFXConfiguration()
            .withSlot1(new Slot1Configs()
                .withKP(0.11)
                .withKI(0.5)
                .withKD(0.0001)
                .withKV(0.12));

    public static final SwerveMotor FRONT_LEFT_DRIVE_MOTOR =
        new TalonFXMotor(
            CANConstants.SWERVE_FRONT_LEFT_DRIVE_ID,
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive))
        );
    public static final SwerveMotor FRONT_LEFT_ANGLE_MOTOR =
        new SparkMaxMotor(SWERVE_FRONT_LEFT_ANGLE_ID);
    public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
        0.2974,
        0.2974
    );

    public static final SwerveMotor FRONT_RIGHT_DRIVE_MOTOR =
        new TalonFXMotor(
            CANConstants.SWERVE_FRONT_RIGHT_DRIVE_ID,
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive))
        );    
        public static final SwerveMotor FRONT_RIGHT_ANGLE_MOTOR =
        new SparkMaxMotor(SWERVE_FRONT_RIGHT_ANGLE_ID);
    public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
        0.2974,
        -0.2974
    );

    public static final SwerveMotor BACK_LEFT_DRIVE_MOTOR =
        new TalonFXMotor(
            CANConstants.SWERVE_BACK_LEFT_DRIVE_ID,
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive))
        );
    public static final SwerveMotor BACK_LEFT_ANGLE_MOTOR =
        new SparkMaxMotor(SWERVE_BACK_LEFT_ANGLE_ID);
    public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
        -0.2974,
        0.2974
    );

    public static final SwerveMotor BACK_RIGHT_DRIVE_MOTOR =
        new TalonFXMotor(
            CANConstants.SWERVE_BACK_RIGHT_DRIVE_ID,
            driveConfig.withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive))
        );
    public static final SwerveMotor BACK_RIGHT_ANGLE_MOTOR =
        new SparkMaxMotor(SWERVE_BACK_RIGHT_ANGLE_ID);
    public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
        -0.2974,
        -0.2974
    );
}