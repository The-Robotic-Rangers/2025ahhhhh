package frc.robot.commands.Groups;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.Constants.SetPoints;
import frc.robot.subsystems.CoralWrist;
import frc.robot.commands.ElevatorCommands.liftToSource;
import frc.robot.commands.WristCommands.wristToSourceCommand;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class sourceCommandGroup extends ParallelCommandGroup{
    public sourceCommandGroup (CoralWrist coralwrist, elevator elevator) {
        addCommands(
            new liftToSource(elevator), 
            new wristToSourceCommand(coralwrist)//This command sets the wrist to the L1 position using a custom command instead of a RunCommand for better readability and maintainability.
        );
    }
} 
