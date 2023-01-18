package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight.LightMode.SnapMode;
import frc.robot.util.NerdyMath;

public class Limelight {
    private static Limelight m_Instance;

    private NetworkTable table; // Network table to access Lime Light Values

    // Frequently used entries to store
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    public enum LightMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int ledMode;

        private LightMode(int ledMode) {
            this.ledMode = ledMode;
        }

        /**
         * Get the current LED Mode
         * 
         * @return the LED mode as int
         */
        public int getLedMode() {
            return ledMode;
        }

        public enum StreamMode {
            // Side by side, Secondary camera is placed in lower right, Primary camera is
            // placed in lower right
            STANDARD(0), MAIN(1), SECONDARY(2);

            private final int mode;

            private StreamMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Stream mode
             * 
             * @return The Stream Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

        public enum CamMode {
            // Camera feed uses vision processing, camera feed increases exposure and
            // disables vision processing
            VISION(0), DRIVER(1);

            private int mode;

            private CamMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Camera mode
             * 
             * @return The Cam Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

        public enum SnapMode {
            // Enables snapshots, Disables snapshots
            DISABLED(0), ENABLED(1);

            private int mode;

            private SnapMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Camera mode
             * 
             * @return The Cam Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

    }

    // Light mode tracking values
    private LightMode currentState = LightMode.OFF;
    private final LightMode LIGHT_ON = LightMode.ON;
    private final LightMode LIGHT_OFF = LightMode.OFF;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0;

    // distance from the center of the limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = 60.0;

    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    //double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    //double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    public Limelight()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        setLightState(LIGHT_OFF);
    }
    
    /**
     * Are we currently tracking any potential targets
     * 
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        boolean has = NerdyMath.inRange(table.getEntry("tv").getDouble(0), -0.01, 0.01);
        return !has;
    }

    /**
     * Horizontal offset from crosshair to target
     * 
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
     *         degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getXAngle() {
        return tx.getDouble(0);
    }

    /**
     * Vertical offset from crosshair to target
     * 
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
     *         degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getYAngle() {
        return ty.getDouble(0);
    }

    /**
     * Get the area of the vision tracking box
     * 
     * @return Target Area (0% of image to 100% of image)
     */
    public double getArea() {
        return ta.getDouble(0);
    }

    /**
     * Rotation of the object
     * 
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    /**
     * Latency in ms of the pipeline
     * 
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     *         capture latency.
     */
    public double getDeltaTime() {
        return table.getEntry("tl").getDouble(0);
    }

    /**
     * The length of the shortest side of the bounding box in pixels
     * 
     * @return Side length of shortest side of the fitted bounding box (pixels)
     */
    public double getShortLength() {
        return table.getEntry("tshort").getDouble(0);
    }

    /**
     * The length of the longest side of the bounding box in pixels
     * 
     * @return Side length of longest side of the fitted bounding box (pixels)
     */
    public double getLongLength() {
        return table.getEntry("tlong").getDouble(0);
    }

    /**
     * The length of the horizontal side of the box (0-320 pixels)
     * 
     * @return Horizontal side length of the rough bounding box (0 - 320 pixels)
     */
    public double getHorizontalLength() {
        return table.getEntry("thor").getDouble(0);
    }

    /**
     * The length of the vertical side of the box (0-320 pixels)
     * 
     * @return Vertical side length of the rough bounding box (0 - 320 pixels)
     */
    public double getVerticalLength() {
        return table.getEntry("tvert").getDouble(0);
    }

    /**
     * Returns the index of the current vision pipeline (0... 9)
     * 
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public int getPipeIndex() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    /**
     * The X-Coordinates of the tracked box
     * 
     * @return Number array of corner x-coordinates
     */
    public double[] getXCorners() {
        return table.getEntry("tcornx").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * The Y-Coordinates of the tracked box
     * 
     * @return Number array of corner y-coordinates
     */
    public double[] getYCorners() {
        return table.getEntry("tcorny").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * Sets the Lime Light LED's
     * 
     * @param mode - LightMode (On, Off, Blinking, or determined by the pipeline)
     */
    public void setLightState(LightMode mode) {
        currentState = mode;
        table.getEntry("ledMode").setNumber(currentState.getLedMode());
    }

    /**
     * Set the Lime Light Camera Mode
     * 
     * @param mode - VISION enables vision processing and decreases exposure, DRIVER
     *             disables vision processing and increases exposure
     */
    public void setCamMode(LightMode.CamMode mode) {
        table.getEntry("camMode").setNumber(mode.getMode());

    }

    /**
     * Sets the limelights current pipeline
     * 
     * @param pipeline The pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets the layout of the cameras viewed at 10.56.67.11:5800
     * 
     * @param streamMode - STANDARD is side by side, MAIN is Limelight big with
     *                   secondary camera in bottom right, SECONDARY is vice versa
     */
    public void setStreamMode(LightMode.StreamMode streamMode) {
        table.getEntry("stream").setNumber(streamMode.getMode());
    }

    /**
     * Allow the Lime Light to take snapshots so that it can be tuned off the field
     * 
     * @param mode DISABLED turns Snap Mode off, Enabled turns Snap Mode on
     */
    public void takeSnapshots(LightMode.SnapMode mode) {
        table.getEntry("snapshot").setNumber(mode.getMode());

    }

    /**
     * Toggle the current Lime Light LED's between on and off states
     */
    public void toggleLight() {
        currentState = (currentState == LIGHT_ON) ? LIGHT_OFF : LIGHT_ON;
        setLightState(currentState);
    }

    /**
     * Turn the Lime Light LED's off
     */
    public void turnLightOff() {
        setLightState(LIGHT_OFF);
    }

    /**
     * Turn the Lime Light LED's on
     */
    public void turnLightOn() {
        setLightState(LIGHT_ON);
    }

    /**
     * Get the Lime Light Subsystem instance
     * 
     * @return The Lime Light instance
     */
    public static Limelight getInstance() {
        if (m_Instance == null) {
            m_Instance = new Limelight();
        }

        return m_Instance;
    }

    /**
     * Get the distance of the limelight target
     * 
     * @param h1 - The height of the limelight with respect to the floor
     * @param h2 - The height of the target
     * @param a1 - The mounting angle for the limelight
     * @param a2 - The angle between the limelight angle and the target
     * 
     * @return The distance between the robot and the limelight
     */
    public double getDistance(double h1, double h2, double a1, double a2) {
        return (h2 - h1) / Math.abs(Math.tan(Math.toRadians(a1) + Math.toRadians(a2)));
    }

    /**
     * Output diagnostics
     */
    public void outputTelemetry() {
        SmartDashboard.putBoolean("HasTarget", hasValidTarget());
        SmartDashboard.putNumber("Horizontal Offset", getXAngle());
        SmartDashboard.putNumber("Vertical Offset", getYAngle());
        SmartDashboard.putNumber("Area", getArea());
        SmartDashboard.putNumber("Skew", getSkew());
        SmartDashboard.putString("XCorners", Arrays.toString(getXCorners()));
        SmartDashboard.putString("YCorners", Arrays.toString(getYCorners()));
    }

    /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }

  /* public void teleopPeriodic() {

        Update_Limelight_Tracking();

        double steer = m_Controller.getX(Hand.kRight);
        double drive = -m_Controller.getY(Hand.kLeft);
        boolean auto = m_Controller.getAButton();

        steer *= 0.70;
        drive *= 0.70;

        if (auto)
        {
          if (m_LimelightHasValidTarget)
          {
                m_Drive.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
          }
          else
          {
                m_Drive.arcadeDrive(0.0,0.0);
          }
        }
        else
        {
          m_Drive.arcadeDrive(drive,steer);
        } */
}
