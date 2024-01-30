package frc.robot.subsystems.shooter;

//q
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

/**
 * a class that does things
 *
 * @author David Wharton
 * @author lord gre
 */
public class CommandShooter extends Shooter {

    private static CommandShooter instance;

    public static synchronized CommandShooter getInstance() {
        if (instance == null)
            instance = new CommandShooter();
        return instance;
    }

    // spin up, load, deploy, shoot
    private CommandShooter() {
        super();
    }

    /**
     * a command that turns off the shootshoot
     *
     * @author lord gre
     */
    public Command off() {
        return Commands.runOnce(
                () -> setTargetState(ShooterState.Off), this// poop
        );
    }

    /**
     * a command that turns on the loader
     *
     * @author David Wharton
     */
    public Command load() {
        return Commands.runOnce(
                () -> setTargetState(ShooterState.Loading), this);
    };

    /**
     * a command that spins up the shootshoot motor
     *
     * @author DavsetTargetState(ShooterState.SpinningUp), this
     */

    public Command spinUp() {
        return Commands.runOnce(
                () -> setTargetState(ShooterState.SpinningUp), this);
    };

    /**
     * a command that runs the whole shoot command
     *
     * @author David Wharton and lord gre
     */
    public Command shoot() {
        return spinUp().andThen(
                Commands.waitUntil(() -> spunUp())).andThen(
                        load())
                .andThen(
                        Commands.waitSeconds(0.5))
                .andThen(
                        off());
    }
}