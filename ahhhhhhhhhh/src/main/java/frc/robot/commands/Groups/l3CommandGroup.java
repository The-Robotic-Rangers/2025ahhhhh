package frc.robot.commands.Groups;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.Constants.SetPoints;
import frc.robot.subsystems.CoralWrist;
import frc.robot.commands.ElevatorCommands.liftToL3;
import frc.robot.commands.WristCommands.wristToL3Command;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class l3CommandGroup extends ParallelCommandGroup{
    public l3CommandGroup (CoralWrist coralwrist, elevator elevator) {
        addCommands(
            new liftToL3(elevator), 
            new wristToL3Command(coralwrist)//This command sets the wrist to the L1 position using a custom command instead of a RunCommand for better readability and maintainability.
        );
    }
} 
