package frc.robot.commands.ElevatorCommands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class liftToSource extends RunCommand {
    public liftToSource (elevator elevator) {
        super(()-> elevator.setPosition(9.3),elevator);
             //addRequirements(elevator);
                    new RunCommand (() -> elevator.setPosition(9.3), elevator);
            }
        
            
}
