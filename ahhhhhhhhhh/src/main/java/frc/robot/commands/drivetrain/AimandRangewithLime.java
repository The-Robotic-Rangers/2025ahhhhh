package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class AimandRangewithLime extends Command {
  private final SwerveDrivetrain m_drivetrain;

  public AimandRangewithLime(SwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain); // Prevents other commands from using the drivetrain simultaneously
  }

  @Override
  public void execute() {
    // Use Limelight logic for proportional aiming and ranging
    var rot = m_drivetrain.limelight_aim_proportional();
    var xSpeed = m_drivetrain.limelight_range_proportional();
    var ySpeed = 0;

    m_drivetrain.drive(xSpeed, ySpeed, rot, false, true); // Field-relative driving disabled
  }

  @Override
public boolean isFinished() {
    return Math.abs(m_drivetrain.getLimelightError()) < 0.5; // Placeholder for target error threshold
}
}
