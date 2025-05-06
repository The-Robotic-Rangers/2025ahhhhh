package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Coral;

public class CoralStop extends StartEndCommand {
    public CoralStop(Coral coral) {
        super(
            () -> {
                coral.setCoralVoltage(0); // Reverse SparkMax motor for ejection
                coral.setTalonPower(0);  // Reverse Talon SRX motor at 50% power
            },
            () -> {
                coral.setCoralVoltage(0); // Stop SparkMax motor
                coral.setTalonPower(0);   // Stop Talon SRX motor
            },
            (Subsystem) coral
        );
    }
}
