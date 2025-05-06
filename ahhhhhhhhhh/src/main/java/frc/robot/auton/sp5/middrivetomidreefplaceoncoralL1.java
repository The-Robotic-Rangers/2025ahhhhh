package frc.robot.auton.sp5;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.common.CoralonL1;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.elevator.elevator;

public class middrivetomidreefplaceoncoralL1 extends SequentialCommandGroup {
    
        public middrivetomidreefplaceoncoralL1(SwerveDrivetrain drivetrain, RobotContainer container,CoralWrist coralwrist, elevator elevator, Coral coral){
    
            addCommands(
                new CoralonL1(drivetrain, elevator, container, coralwrist, coral));
                new WaitCommand(.5);
        }
    
        
       
    
    }
