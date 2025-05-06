// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;
//import frc.robot.Constants.LimelightConstants;
//import frc.robot.LimelightHelpers;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fin");
  static NetworkTableEntry tx = table.getEntry("tx");
  static NetworkTableEntry ty = table.getEntry("ty");
  static NetworkTableEntry ta = table.getEntry("ta");
  

  /** Creates a new Limelight. */
  public Limelight() {
    setToAprilTagPipeline();
    System.out.println("Limelight initialized!");
  }

  /**
   * changes pipeline to 0 (apriltag pipeline)
   */
  public static void setToAprilTagPipeline() {
    NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("pipeline").setNumber(0);
  }

 /*  public static double glideValue(double speed) {
    double tx = LimelightHelpers.getTX("limelight-fin");

    if (hasValidTargets() == 1) {
      if (tx < 0)
        return speed;
      else if (tx > 0)
        return -speed;
    }
  
    return 0;
  }*/

  public void update() {
    // This method will be called once per scheduler run

    //read values periodically
    //double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    //SmartDashboard.putNumber("tx", x);
    //SmartDashboard.putNumber("ty", y);
    //SmartDashboard.putNumber("ta", area);

    // System.out.println("x:"+ x);
    // System.out.println("y:"+ y);
    // System.out.println("area:"+ area);


    //System.out.println("Auto Estimate: " + autoEstimateDistance());
    // System.out.println("Distance" + calculate);

    //SmartDashboard.putNumber("Distance:", autoEstimateDistance());

  }

  /**
   * Gets the distance from the limelight to the apriltag
   * @param limelightMountAngleDegrees
   * @param goalHeightInches
   * @param limelightLensHeightInches
   * @return distanceFromLimelightToGoalInches
   * 
   */
  public static double getDistanceFromAprilTag(double limelightMountAngleDegrees, 
  double goalHeightInches, double limelightLensHeightInches) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fin");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
    / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  /**
   * Think getDistanceFromAprilTag, but for autonomous
   * @return distanceFromLimelightToGoalInches
   */
  public static double autoEstimateDistance() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-fin");
    //NetworkTableEntry tx = table.getEntry("tx");

    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical
    double limelightMountAngleDegrees = 1.0; // grab later

    // distance from the center of the limelight lens to the floor
    double limelightLensHeightInches = 10;// 12.625 // grab later

    // distance from the targets center to the floor 
    double goalHeightInches = 9;// 13.5 for comp // grab later

    double angelToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angelToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
     / Math.tan(angleToGoalRadians);

    // System.out.println("Tx: " + tx.getDouble(0));
    // System.out.println("Ty: " + ty.getDouble(0));

    // return distance
    return distanceFromLimelightToGoalInches;
  }

  /**
   * 
   * @return 1 if there is a valid target, 0 if not
   */
  public static double hasValidTargets() {
    return NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("tv").getDouble(0);
  }

  /**
   * 
   * @return vertical offset of crosshair to target
   */
  public static double getVerticalOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("ty").getDouble(0);
  }

  /**
   * 
   * @return vertical offset of crosshair to target
   */
  public static double getHorizontalOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("tx").getDouble(0);
  }

public static double getyaxisdistance(){
  return NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("tz").getDouble(0);
}

  /**
   * 
   * @return AprilTag ID as double array
   */
  public static double[] getTagID() {
    return NetworkTableInstance.getDefault().getTable("limelight-fin").getEntry("tid").getDoubleArray(new double[6]);
  }



  /**
   * Distance estimation
   * @param mountAngleDegrees - Degrees backwards the mount is from being perfectly vertical
   * @param lensHeightInches - Height from ground to camera lens
   * @param goalHeightInches - Height from ground to apriltag
   * @return distance
   */
  public static double getDistanceToTarget(double mountAngleDegrees, double lensHeightInches, double goalheightInches) {
    double verticalOffset = getVerticalOffset();
    double angleToGoalDegrees = mountAngleDegrees + verticalOffset;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);// was 3.14159
    double distance = Math.abs((goalheightInches - lensHeightInches) / Math.tan(angleToGoalRadians));
    return distance; 
  }

  public static double getangleToGoalDegrees() {
    double verticalOffset = getVerticalOffset();
    double mountAngleDegrees = 18;
        double angleToGoalDegrees = mountAngleDegrees + verticalOffset;
    return angleToGoalDegrees;
  }

 /* public static double getHypotenuseDistance() {
    double xOffset = getHorizontalOffset(); // tx value
    double yAxisDistance = getyaxisdistance(); // tz value

    return Math.sqrt(Math.pow(xOffset, 2) + Math.pow(yAxisDistance, 2));
}

public void periodic() {
    // Calculate hypotenuse distance
    double hypotenuseDistance = getHypotenuseDistance();

    // Send data to SmartDashboard
    SmartDashboard.putNumber("Hypotenuse Distance to Target", hypotenuseDistance);
}*/

}
