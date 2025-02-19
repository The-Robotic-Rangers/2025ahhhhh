package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class elevatorIOSparkMax implements elevatorIO {

  //public static SparkMax leadMotor;
  //public static SparkMax followerMotor;
  //public RelativeEncoder encoder;

    // Initialize the CANSparkMax motors for main and follower
    public static SparkMax leadMotor = new SparkMax(11, MotorType.kBrushless);//left side
    public static SparkMax followerMotor = new SparkMax(12, MotorType.kBrushless);//right side

  SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
  SparkMaxConfig followerMotorConfig = new SparkMaxConfig();

  static SparkClosedLoopController elevatorPID = leadMotor.getClosedLoopController();
  public RelativeEncoder encoder = leadMotor.getEncoder(); 
  public RelativeEncoder encoder2= followerMotor.getEncoder();

  //Constructor
  public elevatorIOSparkMax() {
    
    // Initialize the CANSparkMax motors for main and follower
    //leadMotor = new SparkMax(11, MotorType.kBrushless);//left side
    //followerMotor = new SparkMax(12, MotorType.kBrushless);//right side

      /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */


      // Configure basic settings of the elevator motor
  leadMotorConfig
  //.inverted(false)
  .idleMode(IdleMode.kBrake)
  .smartCurrentLimit(50)
  .voltageCompensation(12);


  
  /*
  * Configure the closed loop controller. We want to make sure we set the
  * feedback sensor as the primary encoder.
  */
   leadMotorConfig
    .closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control
    .p(.027)
    .i(0)
    .d(0)
    .velocityFF(1.0 / 117.6)
    .outputRange(-1, 1)
    .maxMotion
    // Set MAXMotion parameters for position control
    .maxVelocity(4200)
    .maxAcceleration(6000)
    .allowedClosedLoopError(0.5);

    //apply
    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .voltageCompensation(12)
     .inverted(true)
    .follow(ElevatorConstants.LeftelevatorMotorCanId);
    
    followerMotor.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize the encoder for main
    encoder = leadMotor.getEncoder();
  }

   @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
    //followerMotor.set(-voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
    //SmartDashboard.putNumber("Elevator/Actual Position", encoder.getPosition());


  }

  @Override
  public double getVelocity() {
    // Get the velocity from the encoder
    return encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specified position
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}
