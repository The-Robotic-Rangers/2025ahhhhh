// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveDrivetrain drivetrain;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, SwerveDrivetrain drivetrain) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, .0);  
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, .0);  
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, .00);  // Rotation
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT : Constants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-fin");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-fin") && LimelightHelpers.getFiducialID("limelight-fin") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-fin");

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("Position X", postions[2]); // position along x-axis
      SmartDashboard.putNumber("xspee", xSpeed);
      SmartDashboard.putNumber("xerror", xController.getError());

      double ySpeed = -yController.calculate(postions[0]);
      SmartDashboard.putNumber("Position y", postions[0]); // position along y-axis
      SmartDashboard.putNumber("yspee", ySpeed);
      SmartDashboard.putNumber("yerror", yController.getError());

      double rotValue = -rotController.calculate(postions[4]);
      SmartDashboard.putNumber("rotation", postions[4]); // rotation or yaw relative to target
      SmartDashboard.putNumber("rotvalue", rotValue);
      SmartDashboard.putNumber("roterror", rotController.getError());

      drivetrain.drive(-xSpeed, -ySpeed, -rotValue, false, true);

      if (!rotController.atSetpoint() ||
        !yController.atSetpoint() ||
        !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivetrain.drive(0.0, 0.0, 0.0);
    }
    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0); // Ensure motors are stopped
  }
}

