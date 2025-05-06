package frc.robot.commands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristA;

public class setWristPosition extends Command {
    private final CoralWristA wristSubsystem;
    private final double targetPosition;

    public setWristPosition(CoralWristA wristSubsystem, double targetPosition) {
        this.wristSubsystem = wristSubsystem;
        this.targetPosition = targetPosition;

        addRequirements(wristSubsystem); // Declare dependency on the wrist subsystem
    }

    @Override
    public void initialize() {
        // Log or initialize if necessary
    }

    @Override
    public void execute() {
        // Set the wrist position to the target position
        wristSubsystem.wristAngle(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Consider the command finished if within a tolerance
        return Math.abs(wristSubsystem.getAbsoluteWristPosition() - targetPosition) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up or log if interrupted
        if (interrupted) {
            System.out.println("SetWristPositionCommand was interrupted.");
        }
    }
}
