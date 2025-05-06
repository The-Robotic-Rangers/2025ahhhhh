package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.sensors.Limelight;

public class DrivetrainDriveUsingLimeObjDetection extends Command {
    //private final Limelight limelight; // Reference to the Limelight subsystem
    private final SwerveDrivetrain drivetrain; // Reference to the swerve drivetrain subsystem
    private final double targetRange; // Desired range to the object in meters
    private static final double TOLERANCE = 0.1; // Acceptable tolerance in meters

    public DrivetrainDriveUsingLimeObjDetection(SwerveDrivetrain drivetrain, Limelight limelight, double targetRange) {
        this.drivetrain = drivetrain;
        //this.limelight = limelight;
        this.targetRange = targetRange;

        // Declare subsystem dependencies
        addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        System.out.println("DrivetrainDriveUsingLimeObjDetection initialized");
    }

    @Override
    public void execute() {
        // Retrieve the distance to the target using Limelight
        double currentDistance = Limelight.getDistanceToTarget(1,10,9); // Ensure Limelight provides this method
        double speed = calculateSpeed(currentDistance);

        // Use the drivetrain's drive method with appropriate parameters
        drivetrain.drive(speed, 0.0, 0.0, true, true); // Drive forward, no lateral movement, no rotation
    }

    @Override
    public boolean isFinished() {
        // Check if the robot is within the acceptable range
        double currentDistance = Limelight.getDistanceToTarget(1,10,9);
        return Math.abs(currentDistance - targetRange) <= TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(0.0, 0.0, 0.0, true, true);

        // Log whether the command was interrupted or completed successfully
        if (interrupted) {
            System.out.println("DrivetrainDriveUsingLimeObjDetection interrupted");
        } else {
            System.out.println("DrivetrainDriveUsingLimeObjDetection finished");
        }
    }

    private double calculateSpeed(double currentDistance) {
        // Calculate the error (distance difference) and adjust speed using proportional control
        double error = currentDistance - targetRange;
        double kP = 0.5; // Proportional gain, adjust based on testing

        // Clamp the speed to a safe range [-0.5, 0.5]
        return Math.max(-0.5, Math.min(0.5, -kP * error));
    }
}




