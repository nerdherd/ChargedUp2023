package frc.robot.util;

public class NerdyMath {
    public static double ticksToAngle(double ticks, double ticksPerRotation) {
        return ticks * 360 / ticksPerRotation;
    }

    /**
     * Default value (2048) for Falcon500 built-in encoder and (4096) for CTRE SRX Mag Encoder
     * @param ticks
     * @return
     */
    public static double ticksToAngle(double ticks) {
        return ticks * 360 / 2048;
    }

    public static double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }

    public static double radiansToDegrees(double rad) {
        return rad * 180 / Math.PI;
    }

    public static double angleToTicks(double angle, double ticksPerRotation) {
        return angle * ticksPerRotation / 360;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    public static boolean inRange(double myValue, double min, double max)
    {
        if(myValue >= min && myValue <= max)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}