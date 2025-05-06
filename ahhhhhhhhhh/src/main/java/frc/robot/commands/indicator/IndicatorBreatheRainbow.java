package frc.robot.commands.indicator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indicator;



public class IndicatorBreatheRainbow extends Command {
    private final Indicator indicator;

    public IndicatorBreatheRainbow(Indicator indicator,double timeout) {
        this.indicator = indicator;
        addRequirements(indicator);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Rainbow Breathing Effect");
    }

    @Override
    public void execute() {
        indicator.breatheRainbow();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Rainbow Breathing Effect Ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}
