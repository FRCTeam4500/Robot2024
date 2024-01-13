package frc.robot.hardware;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxMotorController extends CANSparkMax implements EncodedMotorController {
	public SparkMaxMotorController(int deviceID, MotorType type) {
		super(deviceID, type);
	}

    @Override
	public Rotation2d getAngle() {
		return Rotation2d.fromRotations(getEncoder().getPosition());
	}

    @Override
	public void setAngle(Rotation2d position) {
		getPIDController()
			.setReference(position.getRotations(), ControlType.kPosition);
	}

    @Override
	public void setOutput(double output) {
		set(output);
	}

    @Override
	public double getOutput() {
		return get();
	}

    @Override
	public Rotation2d getAngularVelocity() {
		return Rotation2d.fromRotations(getEncoder().getVelocity());
		
	}
	
    @Override
	public void setAngularVelocity(Rotation2d velocity) {
		getPIDController()
			.setReference(
				velocity.getRotations(),
				ControlType.kVelocity
			);
	}

	@Override
	public boolean hasContinuousRotation() {
		return true;
	}

	@Override
	public SparkMaxMotorController configCurrentLimit(int currentLimit) {
		setSmartCurrentLimit(currentLimit);
		return this;
	}

	@Override
	public SparkMaxMotorController configAnglePID(PIDConstants pid) {
		SparkPIDController controller = getPIDController();
		controller.setP(pid.kP);
		controller.setI(pid.kI);
		controller.setD(pid.kD);
		return this;
	}

	@Override
	public EncodedMotorController configVelocityPID(PIDConstants pid) {
		return configAnglePID(pid);
	}

	@Override
	public SparkMaxMotorController configMinAngle(Rotation2d minPosition) {
		setSoftLimit(SoftLimitDirection.kReverse, (float) minPosition.getRotations());
		return this;
	}

	@Override
	public SparkMaxMotorController configMaxAngle(Rotation2d maxPosition) {
		setSoftLimit(SoftLimitDirection.kForward, (float) maxPosition.getRotations());
		return this;
	}

	@Override
	public SparkMaxMotorController configMinOutput(double minOutput) {
		SparkPIDController controller = getPIDController();
		controller.setOutputRange(minOutput, controller.getOutputMax());
		return this;
	}

	@Override
	public SparkMaxMotorController configMaxOutput(double maxOutput) {
		SparkPIDController controller = getPIDController();
		controller.setOutputRange(controller.getOutputMin(), maxOutput);
		return this;
	}

	@Override
	public SparkMaxMotorController configInverted(boolean shouldInvert) {
		super.setInverted(shouldInvert);
		return this;
	}

	@Override
	public SparkMaxMotorController configBrakeOnIdle(boolean shouldBreak) {
        setIdleMode(
          shouldBreak
          ? IdleMode.kBrake
          : IdleMode.kCoast  
        );
		return this;
	}
}