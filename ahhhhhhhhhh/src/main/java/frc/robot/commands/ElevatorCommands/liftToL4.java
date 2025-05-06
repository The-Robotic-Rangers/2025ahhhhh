package frc.robot.commands.ElevatorCommands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class liftToL4 extends RunCommand {
    public liftToL4 (elevator elevator) {
        super(()-> elevator.setPosition(51.2),elevator);
             //addRequirements(elevator);
                    new RunCommand (() -> elevator.setPosition(51.2), elevator);
            }
        
            
}
