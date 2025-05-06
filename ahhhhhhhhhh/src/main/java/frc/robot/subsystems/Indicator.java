/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.interfaces.ICamera;
//import frc.robot.sensors.Limelight;
import frc.robot.Ports;
//import frc.robot.commands.indicator.*;
//import frc.robot.interfaces.ICamera;



// see https://docs.wpilib.org/en/latest/docs/software/actuators/addressable-leds.html

/**
 * The {@code Indicator} class contains fields and methods pertaining to the function of the indicator.
 */
public class Indicator extends SubsystemBase {
private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;
	//private Limelight limelight;
	//private ICamera apriltag_camera;
	//private ICamera object_detection_camera;

	static NetworkTable m_limelighttable = NetworkTableInstance.getDefault().getTable("limelight-fin");
	//m_txEntry = m_limelightTable.getEntry("tx");
    static NetworkTableEntry m_txEntry = m_limelighttable.getEntry("tx");
    static NetworkTableEntry m_tyEntry = m_limelighttable.getEntry("ty");
    static NetworkTableEntry m_taEntry = m_limelighttable.getEntry("ta");
	static NetworkTableEntry m_tzEntry = m_limelighttable.getEntry("tz");

// Store what the last hue of the first pixel is
	private int rainbowFirstPixelHue;
	
	public Indicator(int ledCount) {
		ledBuffer = new AddressableLEDBuffer(ledCount);
		//apriltag_camera = apriltag_camera_in;
		//object_detection_camera = object_detection_camera_in;	
		led = new AddressableLED(Ports.PWM.LED_STRIP);

		// Reuse buffer
		// Default to a length of 60, start empty output
		// Length is expensive to set, so only set it once, then just update data
		//ledBuffer = new AddressableLEDBuffer(60);

		led.setLength(ledBuffer.getLength());

		// Set the data
		led.setData(ledBuffer);

		led.start();
	}

	/*@Override
	public void initDefaultCommand() {  
		// Set the default command for a subsystem here.
		setDefaultCommand(new IndicatorIndicateUsingCamera());
	}*/

	@Override
	public void periodic() {
		// Put code here to be run every loop
	}

	public void updateRainbow()
	{
		// For every pixel
		for (var i = 0; i < ledBuffer.getLength(); i++) {

			// Calculate the hue - hue is easier for rainbows because the color
			// shape is a circle so only one value needs to precess
			final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

			// Set the value
			ledBuffer.setHSV(i, hue, 255, 128);
		}

		// Increase by to make the rainbow "move"
		rainbowFirstPixelHue += 3;

		// Check bounds
		rainbowFirstPixelHue %= 180; 

		// Set the LEDs
		led.setData(ledBuffer);
	}

	public void setHue(int hue)
	{
		// For every pixel
		for (var i = 0; i < ledBuffer.getLength(); i++) {

			// Set the value
			ledBuffer.setHSV(i, hue, 255, 128);
		}

		// Set the LEDs
		led.setData(ledBuffer);
	}

	public void setRed()
	{
		setHue(0);
	}

	public void setYellow()
	{
		setHue(20/*45*/);
	}

	public void setGreen()
	{
		setHue(60);
	}

	public void setBlue()
	{
		setHue(120);
	}

	public void setPurple()
	{
		setHue(160);
	}

	public void Chase(int hue) {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setHSV(i, hue, 255, 128);
			led.setData(ledBuffer);
			try {
				Thread.sleep(50); // Pause to create the chasing effect
			} catch (InterruptedException e) {
				e.printStackTrace();
		         }
	        }
		}

		public void breatheRainbow() {
			for (int hue = 0; hue < 180; hue += 5) { // Cycle through hues
				// Gradually increase brightness (breathing in)
				for (int intensity = 0; intensity <= 255; intensity += 5) {
					for (var i = 0; i < ledBuffer.getLength(); i++) {
						ledBuffer.setHSV(i, hue, intensity, 128);
					}
					led.setData(ledBuffer);
					try {
						Thread.sleep(20); // Smooth effect
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
		
				// Gradually decrease brightness (breathing out)
				for (int intensity = 255; intensity >= 0; intensity -= 5) {
					for (var i = 0; i < ledBuffer.getLength(); i++) {
						ledBuffer.setHSV(i, hue, intensity, 128);
					}
					led.setData(ledBuffer);
					try {
						Thread.sleep(20); // Smooth effect
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}

		public void updateFromCamera() {
			if (m_txEntry == null || m_tyEntry == null || m_taEntry == null || m_tzEntry == null) {
				System.out.println("Limelight entries not initialized!");
				return;
			}
		
			double tx = m_txEntry.getDouble(0.0);
			//double tz = m_tzEntry.getDouble (0.0);
			double ta = m_taEntry.getDouble(0.0); 
		
			if (ta > 3) {
				setGreen(); // Green when tx is 7 and tz (ta) is 1.7
			} else if (tx < 20 && ta > 0) {
				setBlue(); // Blue when tx is between 5 and 
			} else if (ta == 0) {
				updateRainbow(); // No target detected
			} else if (Math.abs(tx) > 20) {
				setYellow(); // Close to centered
			} else {
				setRed(); // Far from target
			}
		
			System.out.println("LED Status updated based on Limelight data.");
		}
		
		/*if (distance_tag > 0.0) { // if we saw something
					if (Math.abs(distance_tag) < 5) { // displays green if in target
						setGreen();
					}
					else if (Math.abs(angle_tag) < 15) { // displays yellow if close to target
						setYellow();
					}
					else { // displays red if far from target 
						setRed();
					}
				} else { // no target, so arbitrarily displays blue 
					setBlue();*/
		
		
					
					/*if (object_detection_camera == null) return;
		
							double distance_note = object_detection_camera.getDistanceToTarget();  // will return 0.0 by convention if no target acquired
							//double angle_note = object_detection_camera.getAngleToTurnToTarget(); // angle call not atomic with distance call, but good enough for this use case
							
							if (distance_note > 0.0) { // if we saw something
								/*if (Math.abs(angle_note) < 5) { // displays green if in target
									setGreen();
								}
								else if (Math.abs(angle_note) < 15) { // displays yellow if close to target
									setYellow();
								}
								else { // displays red if far from target 
									setRed();
								}
								setPurple();
							} else { // no target, so arbitrarily displays blue 
								setBlue();
							}*/
					//////////
					
				}
		
			
