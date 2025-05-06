package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Algae;
//import frc.robot.subsystems.Algae.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

    public class EjectAlgaeCommand extends StartEndCommand {
        public EjectAlgaeCommand(Algae algae) {
        super(
            () -> algae.setAlgaeVoltage(-6), 
            () -> algae.setAlgaeVoltage(0), 
            (Subsystem) algae
        );
    }
}
