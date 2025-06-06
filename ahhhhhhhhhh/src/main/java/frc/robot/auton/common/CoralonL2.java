package frc.robot.auton.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.L2;
import frc.robot.commands.Coral.CoralOut;
import frc.robot.commands.Groups.l2CommandGroup;
//import frc.robot.subsystems.Coral.CoralOut;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.RobotContainer;


public class CoralonL2 extends SequentialCommandGroup {

    public CoralonL2 (SwerveDrivetrain drivetrain, elevator elevator,RobotContainer container, CoralWrist coralWrist, Coral coral) {

        addCommands(  
          new SequentialCommandGroup(

                new MoveForward(drivetrain, container,.5).withTimeout(1.2),

                new WaitCommand(1),

                new l2CommandGroup(coralWrist, elevator).withTimeout(.5), 

                new WaitCommand(.6), 

		            new CoralOut(coral).withTimeout(1)));
              }
}
