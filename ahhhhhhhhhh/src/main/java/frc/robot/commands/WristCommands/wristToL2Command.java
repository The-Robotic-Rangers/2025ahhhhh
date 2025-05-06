package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralWrist;

public class wristToL2Command extends RunCommand {
    //private static final double L2_ANGLE = -5; // Replace with the correct angle

    public wristToL2Command(CoralWrist coralWrist) {
        super(() -> coralWrist.wristAngle(WristConstants.L2_ANGLE), coralWrist);
    }
}