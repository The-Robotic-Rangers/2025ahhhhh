package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class CoralWristA extends SubsystemBase {
    private final SparkMax CoralWrist;
    private final RelativeEncoder wristEncoder;
    private final SparkAbsoluteEncoder absoluteEncoder; // Absolute Encoder instance
    public static final SparkMaxConfig coralwristConfig = new SparkMaxConfig();

    // Tolerance for cross-checking encoders
    private static final double POSITION_TOLERANCE = 0.1;

    public CoralWristA() {
        // Initialize Spark Max and encoders
        CoralWrist = new SparkMax(22, MotorType.kBrushless);
        wristEncoder = CoralWrist.getEncoder();
        absoluteEncoder = CoralWrist.getAbsoluteEncoder(); // Get the absolute encoder

        // Configure the motor settings
        coralwristConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(6);
        coralwristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // Absolute encoder set as primary sensor
            .p(0.55)
            .i(0)
            .d(0)
            .velocityFF(1.0 / 267)
            .outputRange(-1, 1)
            .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);
        coralwristConfig.absoluteEncoder
            .inverted(false) //Set the absolute encoder direction
            .zeroOffset(0.0); // TODO Set the zero offset for the absolute encoder



        // Apply the configuration to the motor
        CoralWrist.configure(coralwristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition(); // Relative encoder position
    }

    public double getAbsoluteWristPosition() {
        return absoluteEncoder.getPosition(); // Absolute encoder position
    }

    public void adjustAngle(double angleRadians) {
        // Adjust the encoder position incrementally
        wristEncoder.setPosition(wristEncoder.getPosition() + angleRadians);
    }

    public void wristAngle(double position) {
        // Closed-loop control based on the target position
        CoralWrist.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }

    public void setWristVoltage(double voltage) {
        // Apply voltage to the wrist motor
        CoralWrist.set(voltage);
    }

    @Override
    public void periodic() {
        // Read relative and absolute encoder positions
        double relativePosition = wristEncoder.getPosition();
        double absolutePosition = absoluteEncoder.getPosition();

        // Send positions to SmartDashboard
        SmartDashboard.putNumber("Wrist Position (Relative)", relativePosition);
        SmartDashboard.putNumber("Wrist Position (Absolute)", absolutePosition);

        // Cross-check for redundancy
        if (Math.abs(relativePosition - absolutePosition) > POSITION_TOLERANCE) {
            SmartDashboard.putString("Wrist Encoder Status", "DISCREPANCY DETECTED");
        } else {
            SmartDashboard.putString("Wrist Encoder Status", "NORMAL");
        }

        // Optionally, combine relative and absolute encoder data for tracking
        double combinedPosition = (relativePosition + absolutePosition) / 2.0;
        SmartDashboard.putNumber("Wrist Position (Combined)", combinedPosition);
    }
}
