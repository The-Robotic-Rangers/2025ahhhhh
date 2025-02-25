package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lime 
{
    static NetworkTable table;

    static double tx;
    static double tz;
    static double targetYaw;

    public static void LimeInit()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight-name");//change name to name of limelight

        table.getEntry("pipeline").setDouble(0.0);
    }

    public static void limePeriodic()
    {
        double[] targetPose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

        tx = targetPose[0];
        tz = targetPose[2];
        targetYaw = targetPose[4];

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("tz", tz);
        SmartDashboard.putNumber("targetYaw", targetYaw);
    }

    public static boolean foundTarget()
    {
        if(table.getEntry("tv").getDouble(0.0) == 1)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }

    public static double tx()//horizontal
    {
        return tx;
    }
    public static double tz()//vertical distance from target
    {
        return tz;
    }
    public static double targetYaw()//angle of target relative to robot
    {
        return targetYaw;
    }
}
