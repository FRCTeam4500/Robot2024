package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
    private static Telescope instance;
    public static synchronized Telescope getInstance() {
        if (instance == null) instance = new Telescope();
        return instance;
    }

    public static final double AMP = 3000;
    public static final double SPEAKER = 3500;

    private TalonSRX extensionMotor;
    private Telescope() {
        extensionMotor = new TalonSRX(12);
        extensionMotor.config_kP(0, 1);
        extensionMotor.configPeakOutputForward(0.6);
        extensionMotor.configPeakOutputReverse(-0.6);
        extensionMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, SPEAKER, AMP));
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
}