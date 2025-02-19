/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.AbsoluteEncoder;

public class Wrist extends SubsystemBase {

  public SparkMax WristMotor =  new SparkMax(WristConstants.Wrist_MotorCanId, MotorType.kBrushless);
  private SparkMaxConfig WristMotorConfig;

  //private SparkClosedLoopController closedLoopController;
  private final AbsoluteEncoder WristAbsEncoder;
  private RelativeEncoder wristrelencoder;
  private SparkClosedLoopController closedLoopController;


  public Wrist() {

      // Configure basic settings of the wrist motor
      WristMotorConfig
      .idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      WristMotorConfig
          .absoluteEncoder
          .zeroOffset(.1);
      WristMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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

          
     //apply
      WristMotor.configure(WristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //get relative encoder
    wristrelencoder = WristMotor.getEncoder();
    WristAbsEncoder= WristMotor.getAbsoluteEncoder();
  }

  @Override

  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Wrist relative encoder", wristrelencoder.getPosition());
    }

  public void setArmSpeed(double speed)
  {
    WristMotor.set(speed);
  }

    public void stop()
  {
    WristMotor.stopMotor();
  }

    public double getAbsoluteEncoderPosition() {
    return WristAbsEncoder.getPosition();
  }

    public void setPosition(double position) {
        closedLoopController.setReference(position, SparkMax.ControlType.kPosition);
  }


}