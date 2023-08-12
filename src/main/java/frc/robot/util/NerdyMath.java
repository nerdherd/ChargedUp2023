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

    /**
     * Re-maps a number from one range to another.
     * 
     * Similar implementation to the arduino 
     * <a href="https://reference.arduino.cc/reference/en/language/functions/math/map/">
     * Math.map()</a> method.
     * 
     * <p>
     * 
     * Example: map(0.75, 0, 1, 1, 0) returns 0.25.
     * 
     * <p>
     * 
     * Does not ensure that a number will stay within the range. 
     * Use {@link #clamp() clamp()} to do so.
     * 
     * @see https://github.com/arduino/ArduinoCore-API/blob/master/api/Common.cpp
     * 
     * @return the re-mapped number
     */
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
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
    
    public static boolean inRangeLess(double myValue, double min, double max)
    {
        if(myValue > min && myValue < max)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}