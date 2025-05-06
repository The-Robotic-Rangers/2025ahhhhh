package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CoralWrist;
import frc.robot.Constants.WristConstants;
//import frc.robot.Constants.WristConstants.*;

public class wristToL1Command extends RunCommand {
     // Replace with the correct angle

    public wristToL1Command(CoralWrist coralWrist) {
        super(() -> coralWrist.wristAngle(WristConstants.L1_ANGLE), coralWrist);
    }
}

