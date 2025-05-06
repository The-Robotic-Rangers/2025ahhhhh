package frc.robot.commands.ElevatorCommands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class liftToL2 extends RunCommand {
    public liftToL2 (elevator elevator) {
        super(()-> elevator.setPosition(5.8),elevator);
             //addRequirements(elevator);
                    new RunCommand (() -> elevator.setPosition(5.8), elevator);
            }
        
            
}
