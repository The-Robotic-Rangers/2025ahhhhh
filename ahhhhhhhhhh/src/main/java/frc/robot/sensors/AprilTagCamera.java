// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.List;

//import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AprilTags;
import frc.robot.interfaces.ICamera;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera implements ICamera {

	//TODO: UPDATE CAM SETTINGS FOR NEW ROBOT
	private static final String DEFAULT_CAM_NAME = "AprilTagCam";
	private static final double CAMERA_HEIGHT_METERS =  Units.inchesToMeters(18);
	private static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(60); // may need to change 
	private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(+20.0); // tilt of our camera (radians)


	static final double APRILTAG_CAMERA_SHOOTER_ALIGNMENT_CORRECTION_DEGREES = 5.0; // apply offset in degrees to compensate for shooter being a bit crooked - TODO adjust as needed

	public AprilTagCamera() {
		super(DEFAULT_CAM_NAME);
	}
	
	public double getDistanceToTarget() {
		//return getDistanceToBestTarget();
		return getDistanceToHighValueTarget();
	}

	public double getAngleToTurnToTarget() {
		//double angle = getAngleToTurnToBestTarget();
		double angle = getAngleToTurnToHighValueTarget();

		if (angle != 0.0) // only if we can see a target, so we continue to return 0.0 if we don't see one.
		{
			angle += APRILTAG_CAMERA_SHOOTER_ALIGNMENT_CORRECTION_DEGREES; // apply offset in degrees to compensate for shooter being a bit crooked
		}

		return angle;
	}


	public double getDistanceToBestTarget() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			if (result.hasTargets() && result.getBestTarget()!=null) {
				double range = PhotonUtils.calculateDistanceToTargetMeters(
					CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
					Units.degreesToRadians(result.getBestTarget().getPitch())
					);
				return Units.metersToInches(range);
			}
			return 0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	}

	public double getYaw() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The yaw of the target in degrees (positive right). */
			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getYaw():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}	
	}

	public double getAngleToTurnToBestTarget()
	{
		return +getYaw();
	}

	public double getPitch() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The pitch of the target in degrees (positive up). */
			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getPitch():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	
	}

	public double getSkew() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The skew of the target in degrees (counter-clockwise positive). */
			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getSkew():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}	
	}

	public int getAprilTagId() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			return result.hasTargets() && result.getBestTarget()!=null? 
				result.getBestTarget().getFiducialId():
				0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0;
		}
	}


	public PhotonTrackedTarget getHighValueTarget(PhotonPipelineResult result) {

		if (result.hasTargets()) {

			List<PhotonTrackedTarget> targets = result.getTargets();

			for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RED_REEF_A 
					|| targetId == AprilTags.RED_REEF_B
					|| targetId == AprilTags.RED_REEF_C
					|| targetId == AprilTags.RED_REEF_D
					|| targetId == AprilTags.RED_REEF_E
					|| targetId == AprilTags.RED_REEF_E
					|| targetId == AprilTags.BLUE_REEF_A 
					|| targetId == AprilTags.BLUE_REEF_B
					|| targetId == AprilTags.BLUE_REEF_C
					|| targetId == AprilTags.BLUE_REEF_D
					|| targetId == AprilTags.BLUE_REEF_E
					|| targetId == AprilTags.BLUE_REEF_E
					|| targetId == AprilTags.BLUE_REEF_F
					)
				{
					return target; // SUPER high value target found - more important than high value only
				}
			}

			for (PhotonTrackedTarget target: targets) 
			{
				int targetId = target.getFiducialId();

				if (targetId == AprilTags.RIGHT_BLUE_SOURCE
					|| targetId == AprilTags.LEFT_BLUE_SOURCE
					|| targetId == AprilTags.LEFT_BLUE_SOURCE
					|| targetId == AprilTags.LEFT_RED_SOURCE
					|| targetId == AprilTags.LEFT_RED_SOURCE)
				{
					return target; // high value target found
				}
			}
		}

		return null; // no high value target found
	}

	public double getDistanceToHighValueTarget() {
		try
		{
			PhotonPipelineResult result = getLatestResult();

			if (result.hasTargets() && getHighValueTarget(result)!=null) {
				double range = PhotonUtils.calculateDistanceToTargetMeters(
					CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
					Units.degreesToRadians(getHighValueTarget(result).getPitch())
					);
				return Units.metersToInches(range);
			}
			return 0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	}

	public double getAngleToTurnToHighValueTarget()
	{
		try
		{
			PhotonPipelineResult result = getLatestResult();

			/* The yaw of the target in degrees (positive right). */
			return result.hasTargets() && getHighValueTarget(result)!=null? 
				getHighValueTarget(result).getYaw():
				0.0;
		}
		catch(Exception x)
		{
			System.out.println("ERROR" + x.toString());
			return 0.0;
		}
	}
}

