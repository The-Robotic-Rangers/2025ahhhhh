// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports for Pathplanner follow commmands/stuff below
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
//import edu.wpi.first.math.estimator.PoseEstimator;
//import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.utils.SwerveUtils;
import frc.robot.Ports;
//import frc.robot.RobotContainer;

//import frc.robot.sensors.LimelightHelpers;

/**
 * The {@code SwerveDrivetrain} class contains fields and methods pertaining to the function of the drivetrain.
 */
public class SwerveDrivetrain extends SubsystemBase {
	public static final double kMaxSpeed = 3.0; // 3 meters per second
	public static final double kMaxAngularSpeed= Math.PI;

	// calibration: manually move wheels so it's facing straight then record the number below, deploy code then enable :)

	public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 1.92; // 1.92 adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 1.38; // 1.38 adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = -.95; // -.95 adjust as needed so that virtual (turn) position of wheel is zero when straight
	public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 2.68; // 2.6 adjust as needed so that virtual (turn) position of wheel is zero when straight

	public static final int GYRO_ORIENTATION = -1; // might be able to merge with kGyroReversed

	public static final double FIELD_LENGTH_INCHES = 54*12+1; // 54ft 1in
	public static final double FIELD_WIDTH_INCHES = 26*12+7; // 26ft 7in

	// turn settings
	// NOTE: it might make sense to decrease the PID controller period below 0.02 sec (which is the period used by the main loop)
	static final double TURN_PID_CONTROLLER_PERIOD_SECONDS = .01; // 0.01 sec = 10 ms 	
	
	static final double MIN_TURN_PCT_OUTPUT = 0.1; // 0.1;
	static final double MAX_TURN_PCT_OUTPUT = 0.2; // 0.4;
	
	static final double TURN_PROPORTIONAL_GAIN = 0.001; // 0.01;
	static final double TURN_INTEGRAL_GAIN = 0.0;
	static final double TURN_DERIVATIVE_GAIN = 0.0; // 0.0001
	
	static final int DEGREE_THRESHOLD = 10; // 3;
	
	private final static int TURN_ON_TARGET_MINIMUM_COUNT = 10; // number of times/iterations we need to be on target to really be on target
	// end turn settings	

	// Create SwerveModules
	private final SwerveModule m_frontLeft = new SwerveModule(
		Ports.CAN.FRONT_LEFT_DRIVING,
		Ports.CAN.FRONT_LEFT_TURNING,
		Ports.Analog.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER);

	private final SwerveModule m_frontRight = new SwerveModule(
		Ports.CAN.FRONT_RIGHT_DRIVING,
		Ports.CAN.FRONT_RIGHT_TURNING,
		Ports.Analog.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER);

	private final SwerveModule m_rearLeft = new SwerveModule(
		Ports.CAN.REAR_LEFT_DRIVING,
		Ports.CAN.REAR_LEFT_TURNING,
		Ports.Analog.REAR_LEFT_TURNING_ABSOLUTE_ENCODER);

	private final SwerveModule m_rearRight = new SwerveModule(
		Ports.CAN.REAR_RIGHT_DRIVING,
		Ports.CAN.REAR_RIGHT_TURNING,
		Ports.Analog.REAR_RIGHT_TURNING_ABSOLUTE_ENCODER);

	// The gyro sensor
	private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI); // usign SPI by default, which is what we want.

	//pose estimator
	private final SwerveDrivePoseEstimator m_poseEstimator;

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	private double m_currentTranslationDir = 0.0;
	private double m_currentTranslationMag = 0.0;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.MAGNITUDE_SLEW_RATE);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.ROTATIONAL_SLEW_RATE);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	
	// Odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
		DrivetrainConstants.DRIVE_KINEMATICS,
		Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()),
		new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_rearLeft.getPosition(),
			m_rearRight.getPosition()
		});

	// other variables
	private boolean isTurning;  // indicates that the drivetrain is turning using the PID controller hereunder

	private int onTargetCountTurning; // counter indicating how many times/iterations we were on target

	private PIDController turnPidController; // the PID controller used to turn

	private RobotConfig config;

	/** Creates a new Drivetrain. */
	public SwerveDrivetrain() {

		m_poseEstimator = new SwerveDrivePoseEstimator(DrivetrainConstants.DRIVE_KINEMATICS,//
			Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()), // gyro angle
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			},
			new Pose2d(), // initial pose, will be reset in the constructor
			VecBuilder.fill(0.1, 0.1, 0.1), // state std devs
			VecBuilder.fill(0.5, 0.5, 0.5) // vision std devs
		);
		
		m_frontLeft.calibrateVirtualPosition(FRONT_LEFT_VIRTUAL_OFFSET_RADIANS); // set virtual position for absolute encoder
		m_frontRight.calibrateVirtualPosition(FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS);
		m_rearLeft.calibrateVirtualPosition(REAR_LEFT_VIRTUAL_OFFSET_RADIANS);
		m_rearRight.calibrateVirtualPosition(REAR_RIGHT_VIRTUAL_OFFSET_RADIANS);

		m_frontLeft.resetEncoders(); // resets relative encoders
		m_frontRight.resetEncoders();
		m_rearLeft.resetEncoders();
		m_rearRight.resetEncoders();

		zeroHeading(); // resets gyro

		// sets initial pose arbitrarily
		// Note: the field coordinate system (or global coordinate system) is an absolute coordinate system where a point on the field is designated as the origin.
		// Positive theta is in the counter-clockwise direction, and the positive x-axis points away from your alliance’s driver station wall,
		// and the positive y-axis is perpendicular and to the left of the positive x-axis.
		Translation2d initialTranslation = new Translation2d(Units.inchesToMeters(FIELD_LENGTH_INCHES/2),Units.inchesToMeters(FIELD_WIDTH_INCHES/2)); // mid field
		Rotation2d initialRotation = new Rotation2d(); 
		Pose2d initialPose = new Pose2d(initialTranslation,initialRotation);
		resetOdometry(initialPose);

		//creates a PID controller
		turnPidController = new PIDController(TURN_PROPORTIONAL_GAIN, TURN_INTEGRAL_GAIN, TURN_DERIVATIVE_GAIN);	
		
		turnPidController.enableContinuousInput(-180, 180); // because -180 degrees is the same as 180 degrees (needs input range to be defined first)
		turnPidController.setTolerance(DEGREE_THRESHOLD); // n degree error tolerated

		try{
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (chassisSpeeds) -> driveRobotRelative(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(SwerveModuleConstants.DRIVING_P, SwerveModuleConstants.DRIVING_I, SwerveModuleConstants.DRIVING_D), // Translation PID constants
                    new PIDConstants(SwerveModuleConstants.TURNING_P, SwerveModuleConstants.TURNING_I, SwerveModuleConstants.TURNING_D) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    	);
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(
			Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			});

		calculateTurnAngleUsingPidController();

		//SmartDashboard.putData("Field", field);
		//field.setRobotPose(m_odometry.getPoseMeters());
		// Update the Field2d pose
        //m_robotContainer.getField().setRobotPose (m_odometry.getPoseMeters());

	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(
			Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			},
			pose);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) 
	{
		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;

			if (m_currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
			}
			

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

			if (angleDif < 0.45*Math.PI) {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			}
			else if (angleDif > 0.85*Math.PI) {
				if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					m_currentTranslationMag = m_magLimiter.calculate(0.0);
				}
				else {
					m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			}
			else {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			}

			m_prevTime = currentTime;
			
			xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
			ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
			m_currentRotation = m_rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

		var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	public void drive(double xSpeed, double ySpeed, double angularSpeed) {
		this.drive(xSpeed, ySpeed, angularSpeed, true, false);
	}

	public void driveRobotRelative(ChassisSpeeds speeds){
		this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false,true);
	}
	
	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0 and seeds the turn encoders using the absolute encoders. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	public ChassisSpeeds getChassisSpeeds() {
		return DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(
		  // supplier for chassisSpeed, order of motors need to be the same as the consumer of ChassisSpeed
		  m_frontLeft.getState(), 
		  m_rearLeft.getState(),
		  m_frontRight.getState(),
		  m_rearRight.getState()
		  );
	  }
	
	
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		setModuleStates(
			DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
	  }

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
		m_gyro.setAngleAdjustment(0);
	}

	public void oppositeHeading() {
		m_gyro.reset();
		m_gyro.setAngleAdjustment(180);
	}

	public void blueLeftSubHeading() {
		m_gyro.reset();
		m_gyro.setAngleAdjustment(300);
	}

	public void blueRightSubHeading() {
		m_gyro.reset();
		m_gyro.setAngleAdjustment(60);
	}

	public void redLeftSubHeading() {
		m_gyro.reset();
		m_gyro.setAngleAdjustment(300);
	}

	public void redRightSubHeading() {
		m_gyro.reset();
		m_gyro.setAngleAdjustment(60);
	}

	public void stop()
	{
		drive(0, 0, 0, false, false);

		isTurning = false;
	}

	// Proportional control for Limelight aiming
	public double limelight_aim_proportional() {
		double kP = 3;//.035
		double targetingAngularVelocity = LimelightHelpers.getTX("limelight-fin") * kP;
		targetingAngularVelocity *= kMaxAngularSpeed;
		targetingAngularVelocity *= -1.0; // Invert for correct direction
		return targetingAngularVelocity;
	  }
	
	  // Proportional control for Limelight ranging
	  public double limelight_range_proportional() {
		double kP = 3;//.1
		double targetingForwardSpeed = LimelightHelpers.getTY("limelight-fin") * kP;
		targetingForwardSpeed *= kMaxSpeed;
		targetingForwardSpeed *= -1.0; // Invert for correct direction
		return targetingForwardSpeed;
	  } 

	  public double getLimelightError() {

        // Example implementation, replace with actual Limelight error calculation

        return 0.0;

    }
	/*public void updatePoseEstimation() {
		boolean useMegaTag2 = true; // set to false to use MegaTag1
		boolean doRejectUpdate = false;
	
		if (useMegaTag2 == false) {
			LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fin");
	
			if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
				if (mt1.rawFiducials[0].ambiguity > .7) {
					doRejectUpdate = true;
				}
				if (mt1.rawFiducials[0].distToCamera > 3) {
					doRejectUpdate = true;
				}
			}
			if (mt1.tagCount == 0) {
				doRejectUpdate = true;
			}
	
			if (!doRejectUpdate) {
				m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
				m_poseEstimator.addVisionMeasurement(
					mt1.pose,
					mt1.timestampSeconds
				);
			}
		} else if (useMegaTag2 == true) {
			LimelightHelpers.SetRobotOrientation("fin", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
			LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("fin");
	
			if (Math.abs(m_gyro.getRate()) > 720) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
				doRejectUpdate = true;
			}
			if (mt2.tagCount == 0) {
				doRejectUpdate = true;
			}
			if (!doRejectUpdate) {
				m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
				m_poseEstimator.addVisionMeasurement(
					mt2.pose,
					mt2.timestampSeconds
				);
			}
		}
	}*/

	/** in dash
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees(-1 * GYRO_ORIENTATION * m_gyro.getAngle()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */

    public SwerveDrivePoseEstimator getPoseEstimator() {
        // return the PoseEstimator instance
        return m_poseEstimator;
    }

	public double getTurnRate() {
		return m_gyro.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public SwerveModule getFrontLeftModule()
	{
		return m_frontLeft;
	}

	public SwerveModule getFrontRightModule()
	{
		return m_frontRight;
	}

	public SwerveModule getRearLeftModule()
	{
		return m_rearLeft;
	}

	public SwerveModule getRearRightModule()
	{
		return m_rearRight;
	}

	public AHRS getImu()
	{
		return m_gyro;
	}

	public boolean isTurning(){
		return isTurning;
	}

	// this method needs to be paired with checkTurnAngleUsingPidController()
	public void turnAngleUsingPidController(double angle) {
		// switches to percentage vbus
		stop(); // resets state
		
		double heading = getHeading() - angle;

		//System.out.println("requested heading " + heading);
		
		turnPidController.setSetpoint(heading); // sets the heading

		turnPidController.reset(); // resets controller
		
		isTurning = true;
		onTargetCountTurning = 0;
		//isReallyStalled = false;
		//stalledCount = 0;		
	}

	public void calculateTurnAngleUsingPidController() {	
		if (isTurning) {

			//System.out.println("current heading: " + getHeading());

			double output = MathUtil.clamp(turnPidController.calculate(getHeading()), -MAX_TURN_PCT_OUTPUT, MAX_TURN_PCT_OUTPUT);
			pidWriteRotation(output);
		}
	}

	// This method checks that we are within target up to ON_TARGET_MINIMUM_COUNT times
	// It relies on its own counter
	public boolean tripleCheckTurnAngleUsingPidController() {	
		if (isTurning) {
			boolean isOnTarget = turnPidController.atSetpoint();
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCountTurning++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCountTurning > 0) { // even though we were on target at least once during a previous iteration
					onTargetCountTurning = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (turning).");
				} else {
					// we are definitely turning
				}
			}
			
			if (onTargetCountTurning > TURN_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isTurning = false;
			}
			
			if (!isTurning) {
				System.out.println("You have reached the target (turning).");
				stop();				 
			}
		}
		return isTurning;
	}

	public void pidWriteRotation(double output) {

		//System.out.println("position error: " + turnPidController.getPositionError());
		//System.out.println("raw output: " + output);
		
		// calling disable() on controller will force a call to pidWrite with zero output
		// which we need to handle by not doing anything that could have a side effect 
		if (output != 0 && Math.abs(turnPidController.getPositionError()) < DEGREE_THRESHOLD)
		{
			output = 0;
		}
		if (output != 0 && Math.abs(output) < MIN_TURN_PCT_OUTPUT)
		{
			output = Math.signum(output) * MIN_TURN_PCT_OUTPUT;
		}

		//System.out.println("output: " + output);

		drive(0, 0, output, true, false); 
	}

}