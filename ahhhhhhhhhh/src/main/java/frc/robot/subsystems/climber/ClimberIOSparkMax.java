// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import frc.robot.Configs;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {
  //private final int MOTOR_GEAR_RATIO = 240;

  private SparkMax ClimberMotor1;
  public static final SparkMaxConfig climberConfig = new SparkMaxConfig();
  private RelativeEncoder motorRelativeEncoder;

  public ClimberIOSparkMax() {

  ClimberMotor1 = new SparkMax(15, MotorType.kBrushless);//left side

  climberConfig
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

    // Configure basic settings of the intake motor
    climberConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    //apply
    ClimberMotor1.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorAngle = motorRelativeEncoder.getPosition();
    inputs.motorVoltage = ClimberMotor1.getBusVoltage();
    inputs.motorCurrent = ClimberMotor1.getOutputCurrent();
  }

  @Override
  public void setMotorVoltage(double volts) {
    ClimberMotor1.setVoltage(volts);
  }

  public void stopMotor() {
    ClimberMotor1.stopMotor();
  }
}
