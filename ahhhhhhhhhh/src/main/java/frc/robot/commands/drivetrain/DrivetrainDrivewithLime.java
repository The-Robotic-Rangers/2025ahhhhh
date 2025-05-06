package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DrivetrainDrivewithLime extends Command {
    private final float[] leftCommand;
    private final float[] rightCommand;
    private final NetworkTable limelightTable;

    private static final float KpAim = -0.1f;
    private static final float KpDistance = -0.1f;
    private static final float minAimCommand = 0.05f;

    public DrivetrainDrivewithLime(float[] leftCommand, float[] rightCommand) {
        this.leftCommand = leftCommand;
        this.rightCommand = rightCommand;
        this.limelightTable = NetworkTableInstance.getDefault().getTable("fin");

        // Declare subsystem dependencies here if necessary
    }

    @Override
    public void execute() {
        double tx = limelightTable.getEntry("tx").getDouble(0.0); // Horizontal offset
        double ty = limelightTable.getEntry("ty").getDouble(0.0); // Vertical offset

        double headingError = -tx;
        double distanceError = -ty;
        double steeringAdjust = 0.0;

        if (tx > 1.0) {
            steeringAdjust = KpAim * headingError - minAimCommand;
        } else if (tx < -1.0) {
            steeringAdjust = KpAim * headingError + minAimCommand;
        }

        double distanceAdjust = KpDistance * distanceError;

        leftCommand[0] += steeringAdjust + distanceAdjust;
        rightCommand[0] -= steeringAdjust + distanceAdjust;
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}