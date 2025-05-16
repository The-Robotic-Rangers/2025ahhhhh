// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final Mode currentMode = Mode.REAL;

	public static final String LOG_DIRECTORY = "/home/lvuser/logs";
	public static final long MIN_FREE_SPACE = 1024 * 1024 * 100; // 100 MB
  
	public static final double RPM_TO_RAD_PER_SEC = 2 * Math.PI / 60;
  
	public static enum Mode {
	  /** Running on a real robot. */
	  REAL,
  
	  /** Running a physics simulator. */
	  SIM,
  
	  /** Replaying from a log file. */
	  REPLAY
	}

	public static class AprilTags {

		// facing the field elements (or from the driver station for the stages)
		public static final int RIGHT_RED_SOURCE = 1;//CORAL Loading area
		public static final int LEFT_RED_SOURCE = 2;//CORAL Loading area
		public static final int RED_PROCESSOR = 3;
		public static final int RED_SIDE_BLUE_BARGE = 4; // 
		public static final int RED_SIDE_RED_BARGE = 5; // 
		public static final int RED_REEF_A = 6; // 
		public static final int RED_REEF_B= 7; // 
		public static final int RED_REEF_C= 8;
		public static final int RED_REEF_D= 9;
		public static final int RED_REEF_E = 10;
		public static final int RED_REEF_F = 11;
		public static final int LEFT_BLUE_SOURCE = 12;
		public static final int RIGHT_BLUE_SOURCE = 13;
		public static final int BLUE_SIDE_BLUE_BARGE = 14;
		public static final int BLUE_SIDE_RED_BARGE = 15;
		public static final int BLUE_PROCESSOR = 16;
		public static final int BLUE_REEF_A = 17; // 
		public static final int BLUE_REEF_B= 18; // 
		public static final int BLUE_REEF_C= 19;
		public static final int BLUE_REEF_D= 20;
		public static final int BLUE_REEF_E = 21;
		public static final int BLUE_REEF_F = 22;
	
		public AprilTags() {
		}
	}
	public static final class DrivetrainConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double MAX_SPEED_METERS_PER_SECOND = 4.0; //4.42; //4.8;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

		public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
		public static final double MAGNITUDE_SLEW_RATE = 1.8; // 2.0; //1.8; // percent per second (1 = 100%)
		public static final double ROTATIONAL_SLEW_RATE = 2.0; // 20.0; //2.0; // percent per second (1 = 100%)

		// Chassis configuration
		public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(26);
		
		// Distance between centers of right and left wheels on robot
		public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24);
		
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
				new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
				new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
				new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

		public static final boolean kGyroReversed = false;
	}

	public static final class SwerveModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
		public static final double WHEEL_DIAMETER_METERS = 0.1016;
		public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
		public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
		public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)	/ DRIVING_MOTOR_REDUCTION;

		public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
		public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

		public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative encoder and Through Bore (or Thrifty in our case) absolute encoder - 150.0 / 7.0

		public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI) / TURNING_MOTOR_REDUCTION ; // radians, per rotation
		public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI) / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

		public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
		public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

		public static final double DRIVING_P = 0.04;
		public static final double DRIVING_I = 0;
		public static final double DRIVING_D = 0;
		public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
		public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

		public static final double TURNING_P = 1.0; // 1.0 might be a bit too much - reduce a bit if needed
		public static final double TURNING_I = 0;
		public static final double TURNING_D = 0;
		public static final double TURNING_FF = 0;
		public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

		public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
		public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

		public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; //50; // amps
		public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
	}

	public static final class AutoConstants {
		public static final double MAX_SPEED_METERS_PER_SECOND = 3.0; //4.42; //3.0;
		public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
		public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

		public static final double HIGH_SPEED_METERS_PER_SECOND = 5.0;

		public static final double REDUCED_SPEED_METERS_PER_SECOND = 2.0; //4.42; //3.0;

		public static final double X_CONTROLLER_P = 1;
		public static final double Y_CONTROLLER_P = 1;
		public static final double THETA_CONTROLLER_P = 1;
		
		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
			MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
	}

	public static final class NeoMotorConstants {
		public static final double FREE_SPEED_RPM = 5676;
	}

	public static final class ElevatorConstants {
		public static final int LeftelevatorMotorCanId = 11;
		public static final int RightelevatorMotorCanID =12;
	
		public static final class ElevatorSetpoints {
		  public static final int kFeederStation = 0;
		  public static final int kLevel1 = 0;
		  public static final int kLevel2 = 0;
		  public static final int kLevel3 = 100;
		  public static final int kLevel4 = 150;
	  }
	}

public static final class WristConstants {
		public static final int Wrist_MotorCanId = 14;

		public static final double PROCESSOR_ANGLE = 0;
		public static final double SOURCE_ANGLE = -1.7;//.15
		public static final double L1_ANGLE = -5;
		public static final double L2_ANGLE = -5;
		public static final double L3_ANGLE = -5;//0.225;
		public static final double L4_ANGLE = 0;
		public static final double TOP_ALGAE_ANGLE = 0;
	  }

      public static final class SetPoints{
		public final double PROCESSOR_HEIGHT = 0;
		public final double SOURCE_HEIGHT = 11.5;
		public final static double L1_HEIGHT = 3;
		public final double L2_HEIGHT = 8;
		public final double L3_HEIGHT = 30;
		public final double L4_HEIGHT = 64
		;
		public final double TOP_ALGAE_HEIGHT = 40;
	  }


// Auto constants
public static final double X_REEF_ALIGNMENT_P = .5;//3.3
public static final double Y_REEF_ALIGNMENT_P = .5;//3.3
public static final double ROT_REEF_ALIGNMENT_P = .005;//.058

public static final double ROT_SETPOINT_REEF_ALIGNMENT = -6
;  // Rotation RY
public static final double ROT_TOLERANCE_REEF_ALIGNMENT = .5;
public static final double X_SETPOINT_REEF_ALIGNMENT = -.01;//tz where - values increase as robot moves away from tag
public static final double X_TOLERANCE_REEF_ALIGNMENT = .02;//.02
public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHT = .28; //tx value where it moves robot horizontally relative to tag (+ to right and - to left as we are looking at it in front of us)
public static final double Y_SETPOINT_REEF_ALIGNMENT_LEFT = -.06; //tx value where it moves robot horizontally relative to tag (+ to right and - to left as we are looking at it in front of us)
public static final double Y_TOLERANCE_REEF_ALIGNMENT = .05;//.02

public static final double DONT_SEE_TAG_WAIT_TIME = 1;//1
public static final double POSE_VALIDATION_TIME = 0.1

;//.5

public static final double MAX_ROT_VELOCITY = 1.0; // 3.0//Allow faster rotational speed
public static final double MAX_ROT_ACCELERATION = 1.0; //2.0// Allow quicker acceleration

public static class LimelightConstants {
	public static final double mountAngleDegrees = 5; // grab later
	public static final double lensHeightInches = 12.5; // grab later
	public static final double goalHeightInches = 13.125; // grab later
}

}
