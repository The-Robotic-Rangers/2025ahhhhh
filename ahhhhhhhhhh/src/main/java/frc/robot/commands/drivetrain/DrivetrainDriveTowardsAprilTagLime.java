package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.Constants;

/**
 * Command to drive towards an AprilTag using a Limelight.
 */
public class DrivetrainDriveTowardsAprilTagLime extends Command {

    private SwerveDrivetrain drivetrain;

    public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;
    public final static int TURN_USING_CAMERA_ON_TARGET_MINIMUM_COUNT = 10;
    private final double VISION_DES_RANGE_m = 20; // TODO: Adjust this value as needed

    private int onTargetCountTurningUsingCamera;

    public DrivetrainDriveTowardsAprilTagLime(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("DrivetrainTurnUsingCamera: initialize");
        onTargetCountTurningUsingCamera = 0;
    }

    @Override
    public void execute() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

        // Distance calculation using Limelight angle and target height
        double targetHeight = 8.75; // Adjust based on actual target height
        double limelightHeight = 10; // Adjust to real Limelight mounting height
        double limelightAngle = 0.0; // Adjust to real mounting angle
        double angleToTargetRad = Math.toRadians(ty + limelightAngle);
        double distanceToTarget = (targetHeight - limelightHeight) / Math.tan(angleToTargetRad);

        // Driving logic using Limelight values
        drivetrain.drive(
            (VISION_DES_RANGE_m - distanceToTarget) * Constants.SwerveModuleConstants.DRIVING_P * Constants.DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
            0,
            tx / 90.0, // Normalized value for turning
            false, true
        );
    }

    @Override
    public boolean isFinished() {
        return !tripleCheckTurnUsingCamera();
    }

    public boolean tripleCheckTurnUsingCamera() {
        boolean isTurningUsingCamera = true;
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);

        boolean isOnTarget = Math.abs(tx) < 3.0; // Angle threshold in degrees, adjust as needed

        if (isOnTarget) {
            onTargetCountTurningUsingCamera++;
        } else {
            if (onTargetCountTurningUsingCamera > 0) {
                onTargetCountTurningUsingCamera = 0;
                System.out.println("Triple-check failed (turning using camera).");
            }
        }

        if (onTargetCountTurningUsingCamera > TURN_USING_CAMERA_ON_TARGET_MINIMUM_COUNT) {
            isTurningUsingCamera = false;
        }

        if (!isTurningUsingCamera) {
            System.out.println("You have reached the target (turning using camera).");
        }

        return isTurningUsingCamera;
    }

    @Override
public void end(boolean interrupted) {
    System.out.println("DrivetrainTurnUsingCamera: end");
    drivetrain.stop();
}
}
