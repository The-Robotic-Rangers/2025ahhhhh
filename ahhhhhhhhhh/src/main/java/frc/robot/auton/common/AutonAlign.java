package frc.robot.auton.common;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Coral.CoralOut;
//import frc.robot.commands.Coral.CoralStop;
//import frc.robot.commands.Groups.l2CommandGroup;
import frc.robot.commands.Groups.l3CommandGroup;
import frc.robot.commands.Groups.processorCommandGroup;
import frc.robot.commands.drivetrain.AlignToReefTagRelative;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.elevator.elevator;


public class AutonAlign extends SequentialCommandGroup {

    public AutonAlign(SwerveDrivetrain drivetrain, elevator elevator, RobotContainer container, CoralWrist coralWrist, Coral coral ) {
    addCommands(  
        new SequentialCommandGroup(
        new AlignToReefTagRelative(false, drivetrain).withTimeout(1.5),
        new WaitCommand(3)),
        new l3CommandGroup(coralWrist, elevator).withTimeout(.5), 
        new WaitCommand(.5),
        new CoralOut(coral).withTimeout(.5),
        new WaitCommand(.5),
        //new CoralStop(coral),
        new MoveForward(drivetrain, container, .3).withTimeout(1.2),
        new processorCommandGroup(coralWrist, elevator).withTimeout(.5)
    );
    
        }
}

