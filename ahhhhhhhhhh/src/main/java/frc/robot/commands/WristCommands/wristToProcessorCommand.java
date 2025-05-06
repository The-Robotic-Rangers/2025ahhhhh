package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralWrist;

public class wristToProcessorCommand extends RunCommand {
    //private static final double L4_ANGLE = -5; // Replace with the correct angle

    public wristToProcessorCommand(CoralWrist coralWrist) {
        super(() -> coralWrist.wristAngle(WristConstants.L4_ANGLE), coralWrist);
    }
}
