package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Coral;

public class CoralIn extends StartEndCommand {
    public CoralIn(Coral coral) {
        super(
            () -> {
                coral.setCoralVoltage(6); // Run SparkMax motor
                coral.setTalonPower(-0.5); // Run Talon SRX motor at 50% power
            },
            () -> {
                coral.setCoralVoltage(0); // Stop SparkMax motor
                coral.setTalonPower(0);   // Stop Talon SRX motor
            },
            (Subsystem) coral
        );
    }
}
