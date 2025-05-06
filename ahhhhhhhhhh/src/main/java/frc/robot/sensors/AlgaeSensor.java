package frc.robot.sensors;


import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;



public class AlgaeSensor extends SubsystemBase {

    /* Set ID in web interface http://172.22.11.2:5812/ */
    private TimeOfFlight AlgaeDistance = new TimeOfFlight(Ports.TOF.AlgaeSensor);

     /*
     * Set the distance to the note to be considered load position.'
     * Measure in (mm) to determine an appropriate value.
     */
    public int AlgaeDistanceCheck = 375;

    /* Constructor */
    public AlgaeSensor() {
        /*
         * Initialize the sensor, and '.setRangingMode(RangingMode.Short)' foDr this
         * usage.
         *
         * | Sample value | Time |
         * |---------------|--------|
         * | 1 | 20 ms |
         * | 2 | 33 ms |
         * | 3 | 50 ms |
         * | 4 (default) | 100 ms |
         * | 5 | 200 ms |

         *****************************/
        // The refresh time at Lake City was 1, i.e. 20ms
        AlgaeDistance.setRangingMode(RangingMode.Short, 1);
    }

    public double getAlgaeDistance() {
        /* Gets the distance from the sensor to the nearest edge of the note */
        return AlgaeDistance.getRange(); // return NoteDistance.
    }

    public boolean isAlgaeLoaded() {
        /* Returns true if the note is loaded, false if not */
        return AlgaeDistance.getRange() < AlgaeDistanceCheck; // return NoteDistance.
    }

    @Override
    public void periodic() {

        // Send Algae Distance to SmartDashboard
        SmartDashboard.putNumber("Algae Distance", AlgaeDistance.getRange());

        // Send Algae Loaded status to SmartDashboard
        SmartDashboard.putBoolean("Algae Loaded", isAlgaeLoaded());

        


        // This method will be called once per scheduler run
        // Only needed for diagnostics
        // Shuffleboard.getTab("Algae").add("Algae Distance", AlgaeDistance.getRange());
        // This is not working as expected. Code crashes saying .add title is already in use, probably because it's being called periodically.
        // Shuffleboard.getTab("Sensors").add("Algae Loaded", isAlgaeLoaded());

    }
}
// end of class AlgaeSensorSubsystem