package frc.robot.subsystems.arm2;

import frc.robot.hardware.TalonSRXMotor;

import static frc.robot.CANConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm2 extends SubsystemBase {
    private static Arm2 instance;
    public static synchronized Arm2 getInstance() {
        if (instance == null) instance = new Arm2();
        return instance;
    }

    private TalonSRXMotor extensionMotor;

    private Arm2() {
        extensionMotor = new TalonSRXMotor(ARM_EXTENSION_MOTOR_ID);
    }

    public void setPosition(int position) {
        extensionMotor.set(ControlMode.Position, position);
    }
}//i