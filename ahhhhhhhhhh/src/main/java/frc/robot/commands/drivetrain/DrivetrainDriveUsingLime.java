package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * Command to drive using a Limelight for AprilTag tracking.
 */
public class DrivetrainDriveUsingLime extends Command {

    private SwerveDrivetrain drivetrain;
    private Joystick joystick;

    public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

    public DrivetrainDriveUsingLime(SwerveDrivetrain drivetrain, Joystick joystick) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        
        addRequirements(drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        System.out.println("DrivetrainDriveUsingLimelight: initialize");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        // Retrieve Limelight horizontal offset
        double tx = NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("tx").getDouble(0.0);

        // Drive using joystick input and Limelight target adjustment
        drivetrain.drive(
            -MathUtil.applyDeadband(-joystick.getY(), JOYSTICK_AXIS_THRESHOLD),//made these negative to test
            -MathUtil.applyDeadband(-joystick.getX(), JOYSTICK_AXIS_THRESHOLD),
            -tx / 90.00, // Normalize turn value
            false, true);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
    

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("DrivetrainDriveUsingLimelight: end");
        drivetrain.stop();
    }
}
