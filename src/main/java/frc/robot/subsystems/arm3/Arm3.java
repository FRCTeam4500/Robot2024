package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
    private TalonSRX extensionMotor;
    public Telescope() {
        extensionMotor = new TalonSRX(12);
        extensionMotor.config_kP(0, 1);
        extensionMotor.configPeakOutputForward(0.6);
        extensionMotor.configPeakOutputReverse(-0.6);
    }

    public Command goZero() {
        return Commands.runOnce(
            () -> extensionMotor.set(ControlMode.Position, 100), this
        );
    }

    public Command goAmp() {
        return Commands.runOnce(
            () -> extensionMotor.set(ControlMode.Position, 3000), this
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> extensionMotor.getSelectedSensorPosition(), null);
    }
}