package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralWrist;

public class wristToL3Command extends RunCommand {
    //private static final double L3_ANGLE = -5; // Replace with the correct angle

    public wristToL3Command(CoralWrist coralWrist) {
        super(() -> coralWrist.wristAngle(WristConstants.L3_ANGLE), coralWrist);
    }
}
