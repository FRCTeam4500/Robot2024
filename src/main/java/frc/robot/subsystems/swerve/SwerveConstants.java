package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.hardware.EncodedMotorController;
import frc.robot.hardware.SparkMaxMotorController;
import static frc.robot.CANConstants.*;

public class SwerveConstants {
    // TODO: Change these!
    /** Drive rotations per motor rotation */
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

    // public static final TalonFXConfiguration driveConfig =
    //     new TalonFXConfiguration()
    //         .withSlot1(new Slot1Configs()
    //             .withKP(0.11)
    //             .withKI(0.5)
    //             .withKD(0.0001)
    //             .withKV(0.12));
    // public static final TalonFXConfiguration angleConfig =
    //     new TalonFXConfiguration()
    //         .withMotionMagic(new MotionMagicConfigs()
    //             .withMotionMagicAcceleration(400)
    //             .withMotionMagicCruiseVelocity(750))
    //         .withSlot0(new Slot0Configs()
    //             .withKP(20)
    //             .withKI(0)
    //             .withKD(0.1)
    //             .withKV(0.12));

    public static final EncodedMotorController FRONT_LEFT_DRIVE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_FRONT_LEFT_DRIVE_ID,
        //     driveConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.Clockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_FRONT_LEFT_DRIVE_ID)
            .configVelocityPID(new PIDConstants(1));
    public static final EncodedMotorController FRONT_LEFT_ANGLE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_FRONT_LEFT_ANGLE_ID,
        //     angleConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_FRONT_LEFT_ANGLE_ID)
            .configAnglePID(new PIDConstants(1));
    public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(
        0.2974,
        0.2974
    );

    public static final EncodedMotorController FRONT_RIGHT_DRIVE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_FRONT_RIGHT_DRIVE_ID,
        //     driveConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_FRONT_RIGHT_DRIVE_ID)
            .configVelocityPID(new PIDConstants(1));
    public static final EncodedMotorController FRONT_RIGHT_ANGLE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_FRONT_RIGHT_ANGLE_ID,
        //     angleConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_FRONT_RIGHT_ANGLE_ID)
            .configAnglePID(new PIDConstants(1));
    public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(
        0.2974,
        -0.2974
    );

    public static final EncodedMotorController BACK_LEFT_DRIVE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_BACK_LEFT_DRIVE_ID,
        //     driveConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.Clockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_BACK_LEFT_DRIVE_ID)
            .configVelocityPID(new PIDConstants(1));
    public static final EncodedMotorController BACK_LEFT_ANGLE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_BACK_LEFT_ANGLE_ID,
        //     angleConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_BACK_LEFT_ANGLE_ID)
            .configAnglePID(new PIDConstants(1));
    public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(
        -0.2974,
        0.2974
    );

    public static final EncodedMotorController BACK_RIGHT_DRIVE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_BACK_RIGHT_DRIVE_ID,
        //     driveConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_BACK_RIGHT_DRIVE_ID)
            .configVelocityPID(new PIDConstants(1));
    public static final EncodedMotorController BACK_RIGHT_ANGLE_MOTOR =
        // new TalonFXMotorController(
        //     CANConstants.SWERVE_BACK_RIGHT_ANGLE_ID,
        //     angleConfig.withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive))
        // );
        new SparkMaxMotorController(SWERVE_BACK_RIGHT_ANGLE_ID)
            .configAnglePID(new PIDConstants(1));
    public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(
        -0.2974,
        -0.2974
    );
}