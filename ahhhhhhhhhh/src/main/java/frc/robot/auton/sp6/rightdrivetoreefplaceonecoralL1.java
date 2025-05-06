package frc.robot.auton.sp6;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.trajectories.MoveInReverseAndFortyFiveLeft;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.ElevatorCommands.liftToL1;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.commands.WristCommands.wristToL1Command;
import frc.robot.commands.Coral.CoralOut;


public class rightdrivetoreefplaceonecoralL1 extends SequentialCommandGroup{
	
	public rightdrivetoreefplaceonecoralL1(SwerveDrivetrain drivetrain, RobotContainer container,CoralWrist coralwrist, elevator elevator, Coral coral) {

		addCommands(
			new MoveInReverseAndFortyFiveLeft(drivetrain, container, 1, 1),
            new WaitCommand(.4),
            new liftToL1(elevator),
            new WaitCommand(.4),
			new wristToL1Command(coralwrist),
            new WaitCommand(.4),
            new CoralOut(coral)

		);
	

}
}