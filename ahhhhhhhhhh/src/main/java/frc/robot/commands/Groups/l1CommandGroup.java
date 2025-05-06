package frc.robot.commands.Groups;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.Constants.SetPoints;
import frc.robot.subsystems.CoralWrist;
import frc.robot.commands.ElevatorCommands.liftToL1;
import frc.robot.commands.WristCommands.wristToL1Command;
import frc.robot.subsystems.elevator.elevator;
//import frc.robot.subsystems.intake.Intake;

public class l1CommandGroup extends ParallelCommandGroup{
    public l1CommandGroup (CoralWrist coralwrist, elevator elevator) {
        addCommands(
            new liftToL1(elevator), 
            new wristToL1Command(coralwrist)//This command sets the wrist to the L1 position using a custom command instead of a RunCommand for better readability and maintainability.
        );
    }
} 






