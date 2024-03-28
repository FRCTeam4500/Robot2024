package frc.robot.subsystems.telescope.real;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.telescope.TelescopeIO;

public class Telescope extends TelescopeIO {
    private TalonSRX extensionMotor;
    private Mechanism2d currentMech;
    private MechanismRoot2d root;
    private MechanismLigament2d telescopeState;
    public Telescope() {
        extensionMotor = new TalonSRX(12);
        extensionMotor.config_kP(0, 1);
        extensionMotor.configPeakOutputForward(0.6);
        extensionMotor.configPeakOutputReverse(-0.4);
        extensionMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 31, 0.1));

        currentMech = Superstructure.getCurrentMech();
        root = currentMech.getRoot("Telescope Root", 0.889, 0.1524);
        telescopeState = new MechanismLigament2d("Telescope State", 0.4064, 75);
        root.append(telescopeState);
    }

    public Command extend(double extension) {
        return Commands.runOnce(() -> extensionMotor.set(ControlMode.Position, extension, DemandType.ArbitraryFeedForward, 0.0625), this);
    }

    public Command coast() {
        return Commands.runOnce(() -> extensionMotor.set(ControlMode.PercentOutput, 0), this);
        // return extend(TelescopeIO.STOW);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Position", () -> extensionMotor.getSelectedSensorPosition(), null);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Extension", extensionMotor.getSelectedSensorPosition());
        telescopeState.setLength(0.000166667 * extensionMotor.getSelectedSensorPosition() + 0.4);
    }

    @Override
    public void fromLog(LogTable table) {}

    @Override
    public MechanismLigament2d getCurrentMech() {
        return telescopeState;
    }
}