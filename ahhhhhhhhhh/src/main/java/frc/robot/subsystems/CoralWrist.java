package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class CoralWrist extends SubsystemBase {
    private final SparkMax CoralWrist;
    private final RelativeEncoder wristEncoder;
    public static final SparkMaxConfig coralwristConfig = new SparkMaxConfig();

    public CoralWrist() {
        CoralWrist = new SparkMax(14, MotorType.kBrushless);
        wristEncoder = CoralWrist.getEncoder();

        // Configure basic settings for the motor
        coralwristConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(6);

        // Configure closed-loop settings
        coralwristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.55)
            .i(0)
            .d(0)
            .velocityFF(1.0 / 267)
            .outputRange(-1, 1)
            .maxMotion
                .maxVelocity(2000)//experiment with this value to adjust speed
                .maxAcceleration(10000)//experiment with this value to adjust acceleration
                .allowedClosedLoopError(0.25);

        // Apply the configuration to the motor
        CoralWrist.configure(coralwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

public void adjustAngle(double angleRadians) {
    CoralWrist.getEncoder().setPosition(CoralWrist.getEncoder().getPosition() + angleRadians);
  }

  //@Override
  public void wristAngle(double position) {
    // System.out.println("Wrist position: " + getWristPosition());
    CoralWrist.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
  }

  //@Override
  public double getWristPosition() {
    return wristEncoder.getPosition();
    // Display encoder position and velocity
    //SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    //SmartDashboard.putNumber("Actual Velocity", WristEncoder.getVelocity());
  }

  //@Override
  public void setWristVoltage(double voltage) {
     System.out.println("Wrist position: " + getWristPosition());
     //SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    CoralWrist.set(voltage);
  }

  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
  }
}
