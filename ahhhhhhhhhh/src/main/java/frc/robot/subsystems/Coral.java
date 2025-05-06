package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private final SparkMax coralIntake;
    public static final SparkMaxConfig coralintakeConfig = new SparkMaxConfig();

    private final WPI_TalonSRX talonMotor; // Adding Talon motor

    public Coral() {
        // Initialize SparkMax motor
        coralIntake = new SparkMax(10, MotorType.kBrushed);
        coralintakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);
        coralIntake.configure(coralintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Talon motor
        talonMotor = new WPI_TalonSRX(20); // Assuming CAN ID is 20, update as needed
        talonMotor.configFactoryDefault(); // Reset Talon to factory defaults
    }

    public void setCoralVoltage(double voltage) {
        coralIntake.setVoltage(voltage);
    }

    // Method to control the Talon motor
    public void setTalonPower(double power) {
        talonMotor.set(ControlMode.PercentOutput, power); // Power should be between -1.0 and 1.0
    }
}
