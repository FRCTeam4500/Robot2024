package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorSPXMotorController extends VictorSPX {
	public VictorSPXMotorController(int port) {
		super(port);
	}

	public void setOutput(double output) {
		set(ControlMode.PercentOutput, output);
	}

	public double getOutput() {
		return getMotorOutputPercent();
	}
}
