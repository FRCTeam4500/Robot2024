package frc.robot.subsystems.telescope.real;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.telescope.TelescopeIO;

public class Telescope extends TelescopeIO {
    private TalonSRX extensionMotor;
    public Telescope() {
        extensionMotor = new TalonSRX(12);
        extensionMotor.config_kP(0, 1);
        extensionMotor.configPeakOutputForward(0.6);
        extensionMotor.configPeakOutputReverse(-0.6);
        extensionMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 31, 0.1));
    }

    public Command extend(double extension) {
        return Commands.runOnce(() -> extensionMotor.set(ControlMode.Position, extension), this);
    }

    public Command coast() {
        return Commands.runOnce(() -> extensionMotor.set(ControlMode.PercentOutput, 0), this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> extensionMotor.getSelectedSensorPosition(), null);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Extension", extensionMotor.getSelectedSensorPosition());
    }

    @Override
    public void fromLog(LogTable table) {}
}