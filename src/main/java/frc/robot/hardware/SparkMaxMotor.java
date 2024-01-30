package frc.robot.hardware;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveMotor;

public class SparkMaxMotor extends CANSparkMax implements SwerveMotor {
	public SparkMaxMotor(int deviceID, MotorType type) {
		super(deviceID, type);
	}

	public SparkMaxMotor(int deviceID) {
		this(deviceID, MotorType.kBrushless);
	}

     
	public Rotation2d getAngle() {
		return Rotation2d.fromRotations(getEncoder().getPosition());
	}

     
	public void setAngle(Rotation2d position) {
		getPIDController()
			.setReference(position.getRotations(), ControlType.kPosition);
	}
     
	public void setOutput(double output) {
		set(output);
	}

	public double getOutput() {
		return get();
	}
     
	public Rotation2d getAngularVelocity() {
		return Rotation2d.fromRotations(getEncoder().getVelocity() / 60.0);
		
	}
	
	public void setAngularVelocity(Rotation2d velocity) {
		getPIDController()
			.setReference(
				velocity.getRotations() * 60,
				ControlType.kVelocity
			);
	}

	public boolean hasContinuousRotation() {
		return true;
	}
	 
	public SparkMaxMotor configCurrentLimit(int currentLimit) {
		setSmartCurrentLimit(currentLimit);
		return this;
	}
	 
	public SparkMaxMotor configAnglePID(PIDConstants pid) {
		SparkPIDController controller = getPIDController();
		controller.setP(pid.kP);
		controller.setI(pid.kI);
		controller.setD(pid.kD);
		return this;
	}
	 
	public SparkMaxMotor configVelocityPID(PIDConstants pid) {
		return configAnglePID(pid);
	}
	 
	public SparkMaxMotor configMinAngle(Rotation2d minPosition) {
		setSoftLimit(SoftLimitDirection.kReverse, (float) minPosition.getRotations());
		return this;
	}

	public SparkMaxMotor configMaxAngle(Rotation2d maxPosition) {
		setSoftLimit(SoftLimitDirection.kForward, (float) maxPosition.getRotations());
		return this;
	}

	public SparkMaxMotor configMinOutput(double minOutput) {
		SparkPIDController controller = getPIDController();
		controller.setOutputRange(minOutput, controller.getOutputMax());
		return this;
	}

	public SparkMaxMotor configMaxOutput(double maxOutput) {
		SparkPIDController controller = getPIDController();
		controller.setOutputRange(controller.getOutputMin(), maxOutput);
		return this;
	}
	 
	public SparkMaxMotor configInverted(boolean shouldInvert) {
		super.setInverted(shouldInvert);
		return this;
	}

	public SparkMaxMotor configBrakeOnIdle(boolean shouldBreak) {
        setIdleMode(
          shouldBreak
          ? IdleMode.kBrake
          : IdleMode.kCoast  
        );
		return this;
	}
}