package frc.robot.subsystems.intake;

//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSparkMax implements IntakeIO {
  SparkMax algaeMotor1;
  SparkMax algaeMotor2;
  SparkMax coralIntake;
  SparkMax coralWrist;
  RelativeEncoder wristEncoder;

    public static final SparkMaxConfig algae1Config = new SparkMaxConfig();
    public static final SparkMaxConfig algae2Config = new SparkMaxConfig();
    public static final SparkMaxConfig coralwristConfig = new SparkMaxConfig();
    public static final SparkMaxConfig coralintakeConfig = new SparkMaxConfig();

  public IntakeIOSparkMax() {
    // find actual motor IDs
    algaeMotor1 = new SparkMax(16, MotorType.kBrushless);
    algaeMotor2 = new SparkMax(17, MotorType.kBrushless);
    coralIntake = new SparkMax(10, MotorType.kBrushless);
    coralWrist = new SparkMax(14, MotorType.kBrushless); 

        algae1Config
        .idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);
        algaeMotor1.configure(algae1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algae2Config
        .idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);
        algaeMotor2.configure(algae2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        coralintakeConfig
        .idleMode(IdleMode.kBrake).smartCurrentLimit(15);
        coralIntake.configure(coralintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // Configure basic settings of the arm motor
      coralwristConfig
      .idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      coralwristConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.55)
          .i(0)
          .d(0)
          .velocityFF(1.0 / 267)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

          
      coralWrist.configure(coralwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       // Configure basic settings of the intake motor
       coralintakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    // ask about gear ratios for all motors
    wristEncoder = coralWrist.getEncoder();

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeMotor1.setVoltage(voltage);
    algaeMotor2.setVoltage(-voltage);
  }

  @Override
  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  @Override
  public void adjustAngle(double angleRadians) {
    coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
  }

  @Override
  public void wristAngle(double position) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
  }

  @Override
  public double getWristPosition() {
    return wristEncoder.getPosition();
    // Display encoder position and velocity
    //SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    //SmartDashboard.putNumber("Actual Velocity", WristEncoder.getVelocity());
  }

  @Override
  public void setWristVoltage(double voltage) {
     System.out.println("Wrist position: " + getWristPosition());
     SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    coralWrist.set(voltage);
  }
}


