package frc.robot.subsystems.messaging;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Messaging extends SubsystemBase implements LoggableInputs {
	private static Messaging instance;
	private StringBuilder messages;
	private boolean isEnabled;

	private Messaging() {
		messages = new StringBuilder("MESSAGES APPEAR BELOW");
        isEnabled = false;
	}

	public void addMessage(String message) {
		if (isEnabled) {
			messages.append("\n").append(message);
		}
	}

	public void setMessagingState(boolean enable) {
		isEnabled = enable;
	}

	public static synchronized Messaging getInstance() {
		if (instance == null) instance = new Messaging();
        return instance;
	}

	@Override
	public void toLog(LogTable table) {
		table.put("Message", messages.toString());
	}

	@Override
	public void fromLog(LogTable table) {}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addStringProperty("Messages", () -> messages.toString(), null);
	}
}
