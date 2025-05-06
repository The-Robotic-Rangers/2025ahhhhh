package frc.robot.auton.common;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Groups.l1CommandGroup;
import frc.robot.commands.Groups.processorCommandGroup;
import frc.robot.commands.drivetrain.AlignToReefTagRelative;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.elevator.elevator;

public class MidCoralonL1 extends SequentialCommandGroup {

    public MidCoralonL1 (SwerveDrivetrain drivetrain,elevator elevator,RobotContainer container, CoralWrist coralWrist, Coral coral) {
      addCommands(  
          new SequentialCommandGroup(
                new AlignToReefTagRelative(true, drivetrain).withTimeout(3),
                new WaitCommand(3),
                new MoveForward(drivetrain, container,2.3),
                new WaitCommand(.5),
                new l1CommandGroup(coralWrist, elevator).withTimeout(.5),
                new WaitCommand(.5),
                new MoveInReverse(drivetrain, container, .2).withTimeout(1.2),
                new processorCommandGroup(coralWrist, elevator)).withTimeout(.5)
                );
    
        }
}
