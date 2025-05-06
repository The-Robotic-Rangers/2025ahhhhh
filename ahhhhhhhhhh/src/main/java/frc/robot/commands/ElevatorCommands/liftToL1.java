package frc.robot.commands.ElevatorCommands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.Constants.WristConstants.*;
//import frc.robot.subsystems.intake.Intake;

public class liftToL1 extends RunCommand {
    public liftToL1 (elevator elevator) {
        super(()-> elevator.setPosition(3),elevator);
             //addRequirements(elevator);
                    new RunCommand (() -> elevator.setPosition(3), elevator);
            }
        
            
}
