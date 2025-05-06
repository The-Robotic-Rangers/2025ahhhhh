package frc.robot.commands.Groups;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.Constants.SetPoints;
import frc.robot.subsystems.CoralWrist;
import frc.robot.commands.ElevatorCommands.liftToL4;
import frc.robot.commands.WristCommands.wristToL4Command;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class l4CommandGroup extends ParallelCommandGroup{
    public l4CommandGroup (CoralWrist coralwrist, elevator elevator) {
        addCommands(
            new liftToL4(elevator), 
            new wristToL4Command(coralwrist)//This command sets the wrist to the L1 position using a custom command instead of a RunCommand for better readability and maintainability.
        );
    }
} 
