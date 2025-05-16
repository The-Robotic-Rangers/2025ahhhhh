// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;

// PID Controler import that was commented out for testing
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// end of PID
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
//import frc.robot.commands.indicator.IndicatorScrollRainbow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.sensors.*;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.climber.*;
import frc.robot.commands.Coral.CoralIn;
import frc.robot.commands.Coral.CoralOut;
import frc.robot.commands.Groups.processorCommandGroup;
import frc.robot.commands.Groups.sourceCommandGroup;
import frc.robot.commands.Groups.l1CommandGroup;
import frc.robot.commands.Groups.l2CommandGroup;
import frc.robot.commands.Groups.l3CommandGroup;
import frc.robot.commands.Groups.l4CommandGroup;
//import frc.robot.commands.Groups.sourceCommandGroup;
import frc.robot.commands.Groups.topAlgaeCommandGroup;
import frc.robot.subsystems.elevator.*;
import frc.robot.commands.Algae.EjectAlgaeCommand;
//import frc.robot.commands.Algae.IntakeAlgaeCommand;
import frc.robot.commands.Algae.AlgaeIntakeUntilSensor;
import frc.robot.commands.drivetrain.*;
import frc.robot.interfaces.ICamera;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.Indicator;
import frc.robot.commands.indicator.*;
//import frc.robot.auton.CustomAuton;
//import frc.robot.auton.*;
import frc.robot.auton.trajectories.*;
import frc.robot.auton.common.AutonAlign;
import frc.robot.auton.common.CoralonL1;
//import frc.robot.auton.common.CoralonL1;
import frc.robot.auton.common.CoralonL1fromLeft;
import frc.robot.auton.common.CoralonL1fromRight;
import frc.robot.auton.common.MidCoralonL1;
import frc.robot.auton.common.TrajectoryGenerationTest;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	// Subsystems

  //public static Subsystem m_coralwrist;
  /*private final double PROCESSOR_HEIGHT = 0;
  private final double SOURCE_HEIGHT = 9.3;//was 10.
  private final double L1_HEIGHT = 3;
  private final double L2_HEIGHT = 5.8;
  private final double L3_HEIGHT = 22.8;//was 25.3
  private final double L4_HEIGHT = 51.2;
  private final double TOP_ALGAE_HEIGHT = 40;*/

  private final double PROCESSOR_ANGLE = 0;
  //private final double SOURCE_ANGLE = -1.7;//.15
  //private final double L1_ANGLE = -5;
  //private final double L2_ANGLE = -5;
  private final double L3_ANGLE = -5;//0.225;
  //private final double L4_ANGLE = 0;
  //private final double TOP_ALGAE_ANGLE = 0;


	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	Command indicatorTimedScrollRainbow; // command to run while starting up and when disabled
	Command indicatorBreatheRainbow; // command to run while starting up and when disabled

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	//public static final String AUTON_CUSTOM = "My Auto";
	public static final String AUTON_SAMPLE_SWERVE = "Sample Swerve";
	public static final String AUTON_SAMPLE_MOVE_FORWARD = "Sample Move Forward";
	public static final String AUTON_SAMPLE_MOVE_IN_REVERSE = "Sample Move In Reverse";
	public static final String AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE = "Sample Move In Gamma Shape";
	public static final String AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE = "Sample Move In L Shape In Reverse";
	public static final String AUTON_ALIGN_TO_REEF_TAG_RELATIVE = "Align to Reef Tag Relative";
	public static final String AUTON_LL_MID_SCORE_L1 = "LL Mid Score L1";
	public static final String AUTON_MID_SCORE_L1 = "Moves Forward and Ejects Coral";
	public static final String AUTON_LEFT_MOVE_TO_REEF = "Left Side Move To Reef";
	public static final String AUTON_RIGHT_MOVE_TO_REEF = "Right Side Move To Reef";
	public static final String MOVES_IN_REVERSE_EJECTS_CORAL = "Moves in Reverse and Ejects Coral";
	public static final String AUTON_TEST_HARDCODED_MOVE_1 = "Test Hardcoded Move 1";
	public static final String AUTON_TEST_HARDCODED_MOVE_2 = "Test Hardcoded Move 2";
	public static final String AUTON_TEST_TRAJECTORY_GENERATION = "Test Trajectory Generation";
	private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();
	/*private String startPosition;

	public static final String GAME_PIECE_NONE = "None";
	public static final String GAME_PIECE_1_CORAL = "1 CORAL";
	//public static final String GAME_PIECE_2_CORALS = "2 CORALS";
	//public static final String GAME_PIECE_3_CORALS = "3 CORALS";
	private String gamePieceSelected;
	private SendableChooser<String> gamePieceChooser = new SendableChooser<>();
	
	public static final String START_POSITION_1 = "Starting Position 1";
	public static final String START_POSITION_2 = "Starting Position 2";
	public static final String START_POSITION_3 = "Starting Position 3";
	public static final String START_POSITION_4 = "Starting Position 4";
	public static final String START_POSITION_5 = "Starting Position 5";
	public static final String START_POSITION_6 = "Starting Position 6";
	//private String startPosition;
	private SendableChooser<String> startPositionChooser = new SendableChooser<>();

	public static final String MAIN_TARGET_SPEAKER = "Reef";
	public static final String MAIN_TARGET_NOWHERE = "Nowhere";
	private String mainTarget;
	private SendableChooser<String> mainTargetChooser = new SendableChooser<>();
	
	public static final String CAMERA_OPTION_USE_ALWAYS = "Always";
	public static final String CAMERA_OPTION_USE_OPEN_LOOP_ONLY = "Open Loop Only";
	public static final String CAMERA_OPTION_USE_CLOSED_LOOP_ONLY = "Closed Loop Only";
	public static final String CAMERA_OPTION_USE_NEVER = "Never";
	private String cameraOption;
	private SendableChooser<String> cameraOptionChooser = new SendableChooser<>();
	
	/*public static final String SONAR_OPTION_USE_ALWAYS = "Always";
	public static final String SONAR_OPTION_USE_RELEASE_ONLY = "Release Only";
	public static final String SONAR_OPTION_USE_GRASP_ONLY = "Grasp Only";
	public static final String SONAR_OPTION_USE_NEVER = "Never";
	private String sonarOption;
	private SendableChooser<String> sonarOptionChooser = new SendableChooser<>();
	
	public static final String CLAW_OPTION_RELEASE = "Release";
	public static final String CLAW_OPTION_DONT_RELEASE = "Don't Release"; 
	private String releaseSelected;
	private SendableChooser<String> releaseChooser = new SendableChooser<>();

	public static final String AUTON_OPTION_JUST_PLACE_CORAL = "Just Place Coral";
	public static final String AUTON_OPTION_LEAVE_COMMUNITY = "Leave Community";
	public static final String AUTON_OPTION_PICKUP_CORAL_AT_SOURCE = "Pickup Coral At Source";
	public static final String AUTON_OPTION_PICKUP_NOTE_AT_MIDLINE = "Pickup Coral from Source";
	public static final String AUTON_OPTION_PICKUP_NOTE_AT_WING = "Pickup Note at Wing";
	public static final String AUTON_OPTION_FEED_NOTE = "Feed Note";*/

	//private String autonOption;
	SendableChooser<String> autonOptionChooser = new SendableChooser<>();
	//private final SendableChooser<Command> autoChooser;

	// sensors
	private final IntakeSensor IntakeSensor = new IntakeSensor(); // for sensing if algae is loaded
    //private final AlgaeSensor algaesensor = new AlgaeSensor();

	private final HMAccelerometer accelerometer = new HMAccelerometer();
	private final ICamera object_detection_camera = new ObjectDetectionCamera();
	private final ICamera apriltag_camera = new AprilTagCamera();
	private final Limelight limelight = new Limelight();
	private final Indicator indicator = new Indicator(63);

	// motorized devices
	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
	private final Climber climber = new Climber(new ClimberIOSparkMax());
    private final elevator elevator = new elevator(new elevatorIOSparkMax());
	public static CoralWrist coralwrist = new CoralWrist();
	public Algae algae = new Algae();
	public Coral coral = new Coral();

	//misc
	private final Field2d field = new Field2d(); //  a representation of the field
	

	// The driver's and copilot's joystick(s) and controller(s)
	/*CommandJoystick joyLeft = new CommandJoystick(Ports.USB.LEFT_JOYSTICK);
	CommandJoystick joyRight = new CommandJoystick(Ports.USB.RIGHT_JOYSTICK);*/
	//CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);

 /** The container for the robot. Contains subsystems, IO devices, and commands. */
 public RobotContainer() {

	   // Configure the SmartDashboard to display the Field2d
	   SmartDashboard.putData("Field", field);

	  //Register Named Commands
       //NamedCommands.registerCommand("Eject Coral", new CoralOut(intake));
       //NamedCommands.registerCommand("Lift to L2", new L2(intake, elevator));

		// choosers (for auton with pathplanner)
		//autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
		
		autonChooser.setDefaultOption("Do Nothing", AUTON_DO_NOTHING);
		//autonChooser.addOption("My Auto", AUTON_CUSTOM);
		autonChooser.addOption("Sample Swerve", AUTON_SAMPLE_SWERVE);
		autonChooser.addOption("Sample Move Forward", AUTON_SAMPLE_MOVE_FORWARD);
		autonChooser.addOption("Sample Move In Reverse", AUTON_SAMPLE_MOVE_IN_REVERSE);
		autonChooser.addOption("Sample Move In Gamma Shape", AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE);
		autonChooser.addOption("Sample Move In L Shape In Reverse", AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE);
		autonChooser.addOption("Align to Reef Tag Relative", AUTON_ALIGN_TO_REEF_TAG_RELATIVE);
		autonChooser.addOption("LL Mid Score L1", AUTON_LL_MID_SCORE_L1);
		autonChooser.addOption("Middle Position Score L1", AUTON_MID_SCORE_L1);
		autonChooser.addOption("Left Side Move To Reef", AUTON_LEFT_MOVE_TO_REEF);
		autonChooser.addOption("Right Side move To Reef", AUTON_RIGHT_MOVE_TO_REEF);
		autonChooser.addOption("Test Trajectory Generation", AUTON_TEST_TRAJECTORY_GENERATION);
		SmartDashboard.putData("Auto choices", autonChooser);

		/*gamePieceChooser.setDefaultOption("None", GAME_PIECE_NONE);
		gamePieceChooser.addOption("1 Note", GAME_PIECE_1_CORAL);
		//gamePieceChooser.addOption("2 Notes", GAME_PIECE_2_CORALS);
		//gamePieceChooser.addOption("3 Notes", GAME_PIECE_3_CORALS);
		SmartDashboard.putData("Game piece choices", gamePieceChooser);

		startPositionChooser.setDefaultOption("Starting Position 1", START_POSITION_1);
		startPositionChooser.addOption("Starting Position 2", START_POSITION_2);
		startPositionChooser.addOption("Starting Position 3", START_POSITION_3);
		startPositionChooser.addOption("Starting Position 4", START_POSITION_4);
		startPositionChooser.addOption("Starting Position 5", START_POSITION_5);
		startPositionChooser.addOption("Starting Position 6", START_POSITION_6);
		SmartDashboard.putData("Start positions", startPositionChooser);

		mainTargetChooser.setDefaultOption("To Nowhere", MAIN_TARGET_NOWHERE);
		mainTargetChooser.addOption("Reef", MAIN_TARGET_NOWHERE);
		SmartDashboard.putData("Main targets", mainTargetChooser);
		
		cameraOptionChooser.setDefaultOption("Always", CAMERA_OPTION_USE_ALWAYS);
		cameraOptionChooser.addOption("Open Loop Only", CAMERA_OPTION_USE_OPEN_LOOP_ONLY);
		cameraOptionChooser.addOption("Closed Loop Only", CAMERA_OPTION_USE_CLOSED_LOOP_ONLY);
		cameraOptionChooser.addOption("Never", CAMERA_OPTION_USE_NEVER);		
		SmartDashboard.putData("Camera options", cameraOptionChooser);
		
		/*sonarOptionChooser.setDefaultOption("Always", SONAR_OPTION_USE_ALWAYS);
		sonarOptionChooser.addOption("Release Only", SONAR_OPTION_USE_RELEASE_ONLY);
		sonarOptionChooser.addOption("Grasp Only", SONAR_OPTION_USE_GRASP_ONLY);		
		sonarOptionChooser.addOption("Never", SONAR_OPTION_USE_NEVER);
		SmartDashboard.putData("Sonar options", sonarOptionChooser);*/
		
		/*releaseChooser.setDefaultOption("Release", CLAW_OPTION_RELEASE);
		releaseChooser.addOption("Don't release", CLAW_OPTION_DONT_RELEASE);
		SmartDashboard.putData("Release options", releaseChooser);/* */

		//autonOptionChooser.setDefaultOption("Just Shoot Note", AUTON_OPTION_JUST_PLACE_CORAL);
		//autonOptionChooser.addOption("Leave Community", AUTON_OPTION_LEAVE_COMMUNITY);
		//autonOptionChooser.addOption("Pickup Coral At Source", AUTON_OPTION_PICKUP_CORAL_AT_SOURCE);
		//autonOptionChooser.addOption("Pickup Note At Wing", AUTON_OPTION_PICKUP_NOTE_AT_WING);
		//autonOptionChooser.addOption("Feed Note", AUTON_OPTION_FEED_NOTE);
		SmartDashboard.putData("Auton options", autonOptionChooser);
		

		// Configure the button bindings

		configureButtonBindings();

		// Configure default commands
		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(driverGamepad.getLeftY(), GAMEPAD_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driverGamepad.getLeftX(), GAMEPAD_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driverGamepad.getRightX(), GAMEPAD_AXIS_THRESHOLD),
					true, true),
				drivetrain));


		indicator.setDefaultCommand(new IndicatorIndicateUsingCamera(indicator)); // default command, only runs when robot is enabled

	    // indicatorTimedScrollRainbow = new IndicatorTimedScrollRainbow(indicator,1);
		//indicatorTimedScrollRainbow.schedule(); // we schedule the command as we are starting up
	    indicatorBreatheRainbow = new IndicatorBreatheRainbow(indicator,1);
	    indicatorBreatheRainbow.schedule(); // we schedule the command as we are starting up
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		// driver (joystick)

		driverGamepad.povUp()
			.onTrue(new DrivetrainZeroHeading(drivetrain));	

			driverGamepad.povDown()
			.onTrue(new DrivetrainOppositeHeading(drivetrain));	

			driverGamepad.povLeft()
			.onTrue(new DrivetrainLeftSubHeading(drivetrain));	

			driverGamepad.povRight()
			.onTrue(new DrivetrainRightSubHeading(drivetrain));

		//joyMain.button(1)
			//.whileTrue(new DrivetrainDriveUsingAprilTagCamera(drivetrain, apriltag_camera, getMainJoystick()));
			//driverGamepad.button(9)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));	
			//.whileTrue(new DrivetrainDriveUsingObjectDetectionCamera(drivetrain, object_detection_camera, getMainJoystick()));
			driverGamepad.button(4)
			.onTrue(new AlignToReefTagRelative(false, drivetrain));


			driverGamepad.button(2)
			//.onTrue(new MoveInLShapeInReverse(drivetrain, this, 3));
			//.whileTrue(new AimandRangewithLime(drivetrain));
			//.onTrue(new IndicatorChase(indicator));
			//.onTrue(new IndicatorBreatheRainbow(indicator,10));
			.onTrue(new AlignToReefTagRelative(true, drivetrain));
			//.onTrue(new DrivetrainDriveUsingLime(drivetrain, driverGamepad));
			//.onTrue(new DrivetrainDriveTowardsAprilTagLime(drivetrain));



			//driverGamepad.button(11)
			//.onTrue(new MoveInGammaShape(drivetrain, this, 3));

		//joyMain.button(5)
			//.onTrue(new MoveForward(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, -90));
			//.onTrue(new MoveInUShapeInReverse(drivetrain, this, 1));

		//joyMain.button(6)
			//.onTrue(new MoveInReverse(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, 90));
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));

		//joyMain.button(7)
			//.whileTrue(new RollerJoystickControl(roller, drivetrain, getMainJoystick()));
		
		//joyMain.button(8)
			//.whileTrue(new NeckJoystickControl(neck, drivetrain, getMainJoystick()));
		
		//joyMain.button(9)
			//.whileTrue(new ShooterJoystickControl(shooter, drivetrain, getMainJoystick()));
		
		//joyMain.button(10)
			//.whileTrue(new ElevatorJoystickControl(elevator, drivetrain, getMainJoystick()));

		//joyMain.button(11)
			//.onTrue(new DrivetrainZeroHeading(drivetrain));
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, apriltag_camera));
		
		//joyMain.button(12)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			//.onTrue(new IndicatorSetGreen(Indicator indicator));
    	Command climbUpCommand =
		new StartEndCommand(() -> climber.setMotorVoltage(6), () -> climber.stopMotor(), climber);
		Command climbDownCommand =
		new StartEndCommand(() -> climber.setMotorVoltage(-6), () -> climber.stopMotor(), climber);
		//Command climbHoldCommand =
		//new StartEndCommand(
		//() -> climber.setMotorVoltage(-0.75), () -> climber.stopMotor(), climber);

		driverGamepad.button(1).whileTrue(climbUpCommand);
		//driverGamepad.button(2).whileTrue(climbHoldCommand);
		driverGamepad.button(3).whileTrue(climbDownCommand);

	// Algae
		Command ejectAlgaeCommand = new EjectAlgaeCommand(algae);
		driverGamepad.button(6).whileTrue(ejectAlgaeCommand);

		Command IntakeAlgaeUntilSensor = new AlgaeIntakeUntilSensor(algae, IntakeSensor);
		driverGamepad.button(5).whileTrue(IntakeAlgaeUntilSensor);

    // Intake coral

	Command CoralIn = new CoralIn(coral);
	driverGamepad.leftTrigger().whileTrue(CoralIn);

	Command coralOutCommand = new CoralOut(coral);
	driverGamepad.rightTrigger().whileTrue(coralOutCommand);
		
    //CoPilot Buttons//
    // Processor state
	Command processorCommandGroup = new processorCommandGroup(coralwrist, elevator);
	copilotGamepad.povDown().onTrue(processorCommandGroup);
   
	//Wrist stuff
	   Command Wristdown =
		new RunCommand(()->coralwrist.wristAngle(L3_ANGLE), coralwrist);
		copilotGamepad.rightBumper().whileTrue(Wristdown);

		Command Wristup =
		new RunCommand(()->coralwrist.wristAngle(PROCESSOR_ANGLE), coralwrist);
		copilotGamepad.povRight().whileTrue(Wristup);
	

    // Source state
	Command sourceCommandGroup = new sourceCommandGroup(coralwrist, elevator);
	copilotGamepad.povLeft().onTrue(sourceCommandGroup);

	Command l1CommandGroup = new l1CommandGroup(coralwrist, elevator);
	copilotGamepad.a().onTrue(l1CommandGroup);

	Command l2CommandGroup = new l2CommandGroup(coralwrist, elevator);
	copilotGamepad.b().onTrue(l2CommandGroup);

	Command L3CommandGroup = new l3CommandGroup(coralwrist, elevator);
	copilotGamepad.y().onTrue(L3CommandGroup);

	Command L4CommandGroup = new l4CommandGroup(coralwrist, elevator);
	copilotGamepad.x().onTrue(L4CommandGroup);

	Command TopAlgaeCommandGroup = new topAlgaeCommandGroup(coralwrist, elevator);
	copilotGamepad.povUp().onTrue(TopAlgaeCommandGroup);

    // Manual lift
   Command manualLift =
        new RunCommand(() -> elevator.setVoltage(-copilotGamepad.getLeftY() * 0.5), elevator);
    // Command manualWrist =
    //     new RunCommand(() -> intake.setWristVoltage(operatorController.getRightY() * 0.25),
    // intake);
    // ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
    copilotGamepad.start().whileTrue(manualLift);

	//Auto Commands Testing
	
		//copilotGamepad.x()
		    //.onTrue(new CoralIntake());
			//.whileTrue(new RollerRelease(roller));
			//.onTrue(new CoralIntake(m_coralintake));
		
		//copilotGamepad.b()
			//.onTrue(new CoralOuttake(m_coralouttake));
			//.whileTrue(new RollerRoll(roller));

		//copilotGamepad.x()
			//.onTrue(new RollerReleaseShortDistance(roller));

		//copilotGamepad.y()
			//.whileTrue(new RollerRollLowRpm(roller));
			//.onTrue(new RollerRollLowRpmUntilNoteSensed(roller, getNoteSensor()));
			//.onTrue(new RollerReleaseShortDistance(roller));
			//.onTrue(new RollerSuperSmartRoll(roller, noteSensor, noteSensorTwo));
			//.onTrue(new RollerRollLowRpmUntilNoteSensed(roller, noteSensor, noteSensorTwo));
			
		//copilotGamepad.back()
			//.onTrue(new DrivetrainAndGyroReset(drivetrain));
			//.onTrue(new AlmostEverythingStop(elevator, neck, roller, shooter));

		//copilotGamepad.start()
			//.onTrue(new AlmostEverythingStop(elevator, neck, roller));
			//.onTrue(new NeckHome(neck));


		//copilotGamepad.leftTrigger()
			//.onTrue(new DrawerRetractWithStallDetection(drawer));
			//.whileTrue(new ShooterTake(shooter));
			//.whileTrue(new ShooterShootHigh(shooter));

		//copilotGamepad.rightTrigger()
			//.onTrue(new DrawerExtendWithStallDetection(drawer));
			//.whileTrue(new RollerRoll(roller));


		//copilotGamepad.povDown()
			//.onTrue(new ElevatorMoveDownWithStallDetection(elevator));
			//.onTrue(new NeckMoveDownWithStallDetection(neck));

		//copilotGamepad.povLeft()
			//.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));
			//.onTrue(new NeckMoveSubWithStallDetection(neck));

		//copilotGamepad.povRight()
			//.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));
			//.onTrue(new NeckMovePodiumWithStallDetection(neck));
			//.onTrue(new NeckMoveFeedNoteWithStallDetection(neck));

		//copilotGamepad.povUp()
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));


		//copilotGamepad.leftBumper()
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.whileTrue(new NeckMoveUsingCamera(neck, apriltag_camera));

		//copilotGamepad.rightBumper()
			//.onTrue(new NeckMoveDownWithStallDetection(neck));
			//.onTrue(new NeckMoveAcrossFieldWithStallDetection(neck));


		//copilotGamepad.leftStick()
			//.onTrue(new RollerTimedRoll(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));			

		//copilotGamepad.rightStick()
			//.onTrue(new RollerTimedRelease(roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));


		//copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD)
			//.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		//copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD)
			//.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		/*copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();*/

		//copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD)
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		//copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD)
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		//copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));
			//.onTrue(new NeckMovePodiumWithStallDetection(neck));

		//copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));
			//.onTrue(new NeckMoveSubWithStallDetection(neck));
			
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		//return autoChooser.getSelected();

		autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);	

		/*gamePieceSelected = gamePieceChooser.getSelected();
		System.out.println("Game piece selected: " + gamePieceSelected);		

		startPosition = startPositionChooser.getSelected();
		System.out.println("Start position: " + startPosition);

		mainTarget = mainTargetChooser.getSelected();
		System.out.println("Main target: " + mainTarget);
		
		cameraOption = cameraOptionChooser.getSelected();
		System.out.println("Camera option: " + cameraOption);
		
		/*sonarOption = sonarOptionChooser.getSelected();
		System.out.println("Sonar option: " + sonarOption);
		
		releaseSelected = releaseChooser.getSelected();
		System.out.println("Release chosen: " + releaseSelected);

		autonOption = autonOptionChooser.getSelected();
		System.out.println("Auton option: " + autonOption);*/

	switch (autonSelected) {
			case AUTON_SAMPLE_SWERVE:
				//return createSwerveControllerCommand(createExampleTrajectory());
				//return new DrivetrainSwerveRelative(drivetrain, this, createExampleTrajectory());
				return new MoveInSShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_FORWARD:
				return new MoveForward(drivetrain, this, 1.5);
				//break;

			case AUTON_SAMPLE_MOVE_IN_REVERSE:
				return new MoveInReverse(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE:
				return new MoveInGammaShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE:
				return new MoveInLShapeInReverse(drivetrain, this, 3);
				//break;
			case AUTON_ALIGN_TO_REEF_TAG_RELATIVE:
				return new AutonAlign(drivetrain, elevator, this, coralwrist, coral);
				//break;

			case AUTON_TEST_HARDCODED_MOVE_1:
				return new CompletelyLeaveCommunity(drivetrain, this);
				//break;
			case AUTON_LEFT_MOVE_TO_REEF:
				return new CoralonL1fromLeft(drivetrain, elevator,this,coralwrist, coral);
				//break;

			case AUTON_RIGHT_MOVE_TO_REEF:
				return new CoralonL1fromRight(drivetrain, elevator, this, coralwrist, coral);
				//break;

			//case AUTON_TEST_HARDCODED_MOVE_2:
			case AUTON_MID_SCORE_L1:
			return new CoralonL1(drivetrain,elevator, this, coralwrist, coral);
			//break;
				//break;*/

			case AUTON_LL_MID_SCORE_L1:
			return new MidCoralonL1(drivetrain, elevator, this, coralwrist, coral);
			//break;
				//break;

			case AUTON_TEST_TRAJECTORY_GENERATION:
				return new TrajectoryGenerationTest(drivetrain, this, object_detection_camera, apriltag_camera);
				//break;*/

			//case AUTON_CUSTOM:
				//return new CustomAuton(gamePieceSelected, startPosition, mainTarget, autonOption, autonOption, autonOption, drivetrain, this, elevator, null, coral);
				//break;

			case AUTON_DO_NOTHING:
				return null;
				//break;
				
			default:
				// nothing
				return null;
				//break;
		} // end switch

	}

	public TrajectoryConfig createFastTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.HIGH_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createSlowTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.REDUCED_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}


	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public TrajectoryConfig createFastReverseTrajectoryConfig() {

		TrajectoryConfig config = createFastTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public Trajectory createExampleTrajectory() {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
			createTrajectoryConfig());

		return exampleTrajectory;
	}
	
	/*public Command createSwerveControllerCommand(Trajectory trajectory) {

		ProfiledPIDController thetaController = new ProfiledPIDController(
			AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory, // trajectory to follow
			drivetrain::getPose, // Functional interface to feed supplier
			DrivetrainConstants.DRIVE_KINEMATICS, // kinematics of the drivetrain
			new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for x position
			new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for y position
			thetaController, // trajectory tracker PID controller for rotation
			drivetrain::setModuleStates, // raw output module states from the position controllers
			drivetrain); // subsystems to require

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose()); // WARNING: https://github.com/REVrobotics/MAXSwerve-Java-Template/issues/13

		field.getObject("trajectory").setTrajectory(trajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
	}*/

	/*public AlgaeSensor getAlgaeSensor()
    {
	return algaesensor;
    }*/

	public IntakeSensor getIntakeSensor() {
		return IntakeSensor;
	}

	public Limelight getLimelight() {
		return limelight;
	}
	
	public Indicator getIndicator()
	{
		return indicator;
	}
	public CoralWrist getCoralWrist()
	{
		return coralwrist;
	}

	public Algae getAlgae()
	{
		return algae;
	}
	public Coral getCoral()
	{
		return coral;
	}

	public Field2d getField()
	{
		return field;
	}

	public HMAccelerometer getAccelerometer()
	{
		return accelerometer;
	}

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}
    public Climber getClimber()
    {
	return climber;
    }
	
	public elevator getElevator()
	{
		return elevator;
	}

    public ICamera getObjectDetectionCamera()
	{
		return object_detection_camera;
	}

	public ICamera getAprilTagCamera()
	{
		return apriltag_camera;
	}

	public XboxController getdriverGamepad()
	{
		return driverGamepad.getHID();
	}

	public XboxController getCopilotGamepad()
	{
		return copilotGamepad.getHID();
	}

	public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}
	
	/*public SendableChooser<String> getGamePieceChooser()
	{
		return gamePieceChooser;
	}

	public SendableChooser<String> getStartPositionChooser()
	{
		return startPositionChooser;
	}

	public SendableChooser<String> getMainTargetChooser()
	{
		return mainTargetChooser;
	}

	public SendableChooser<String> getCameraOptionChooser()
	{
		return cameraOptionChooser;
	}

	/*public SendableChooser<String> getSonarOptionChooser()
	{
		return sonarOptionChooser;
	}

	public SendableChooser<String> getReleaseChooser()
	{
		return releaseChooser;
	}*/

	public SendableChooser<String> getAutonOptionChooser()
	{
		return autonOptionChooser;
	}
}
