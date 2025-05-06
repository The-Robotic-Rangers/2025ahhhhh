package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Algae implements Subsystem {
    private final SparkMax algaeMotor1;
    private final SparkMax algaeMotor2;
    //private final Indicator indicator;

    public static final SparkMaxConfig algae1Config = new SparkMaxConfig();
    public static final SparkMaxConfig algae2Config = new SparkMaxConfig();
    

public Algae() {

    algaeMotor1 = new SparkMax(16, MotorType.kBrushless);
    algaeMotor2 = new SparkMax(17, MotorType.kBrushless);

    algae1Config
        .idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);
    algaeMotor1.configure(algae1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algae2Config
        .idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(15);
    algaeMotor2.configure(algae2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


public void setAlgaeVoltage(double voltage) {

    algaeMotor1.setVoltage(voltage);
    algaeMotor2.setVoltage(voltage);
}
}