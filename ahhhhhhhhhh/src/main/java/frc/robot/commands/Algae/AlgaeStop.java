package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Algae;
//import frc.robot.subsystems.Algae.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

    public class AlgaeStop extends StartEndCommand {
        public AlgaeStop(Algae algae) {
        super(
            

            () -> algae.setAlgaeVoltage(0), 
            () -> algae.setAlgaeVoltage(0), 
            (Subsystem) algae

           
        );
    }
}
