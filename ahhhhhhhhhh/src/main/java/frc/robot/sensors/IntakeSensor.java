// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;

/**
 * The {@code NoteSensor} class contains fields and methods pertaining to the function of the note sensor.
 */
public class IntakeSensor
{
	private DigitalInput digitalInput;

	public IntakeSensor() {
		this.digitalInput = new DigitalInput(Ports.Digital.IntakeSensor);
	}

	/**
	 * Returns the state of the note sensor.
	 *
	 * @return the current state of the note sensor.
	 */
	public boolean isEnergized() {
		return digitalInput.get();
	}
}
