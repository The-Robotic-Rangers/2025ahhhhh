
package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.IntakeSensor;
import frc.robot.subsystems.Algae;

/**
 *
 */
public class AlgaeIntakeUntilSensor extends Command {

	private Algae algae;
	private IntakeSensor intakesensor;

	public AlgaeIntakeUntilSensor(Algae algae, IntakeSensor intakesensor) {
		this.algae = algae;
		this.intakesensor = intakesensor;
		
		addRequirements(algae); // Prevents other commands from using the algae subsystem simultaneously
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeIntakeUntilAlgaeSensed: initialize");
		algae.setAlgaeVoltage(12);
       // algae.setAlgaeVoltage(0); // Start the algae intake mechanism by setting the voltage to 12V
	}

	@Override
	public boolean isFinished() {
		if (intakesensor.isEnergized() == false) { // Stop when sensor detects algae (false = tripped)
			algae.setAlgaeVoltage(0); // Stop the motors
			
		} else {
			algae.setAlgaeVoltage(12); // Keep the motors running
		}
		return false; // Continue running the command
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("AlgaeIntakeUntilAlgaeSensed: end");
		algae.setAlgaeVoltage(0);
        //algae.setAlgaeVoltage(0);

	}

}
