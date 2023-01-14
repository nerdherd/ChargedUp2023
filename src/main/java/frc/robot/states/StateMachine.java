package frc.robot.states;

import java.util.ResourceBundle.Control;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.util.NerdyMath;

public class StateMachine {

    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    //PhotonCamera camera = new PhotonCamera("photonvision");
    Vision apriltagCamera;// = new Vision();
    //NetworkTable tableApriltag = NetworkTableInstance.getDefault().getTable("limelight");
    Limelight objDetectCamera;// = new Limelight();

    //XboxController xboxController = new XboxController(0);
    //Joystick stick = new Joystick(0);;
    
    private final double wheelDiameter = 6.0;
    private final double encoderTicksPerRotation = 1440.0; // 360 encode x SRX 4:1
    private double drivePower = 0.5;

    // instantiate the motors
    /*WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
    WPI_TalonSRX rightMotor = new WPI_TalonSRX(0);
    WPI_TalonSRX leftFollower = new WPI_TalonSRX(3);
    WPI_TalonSRX rightFollower = new WPI_TalonSRX(2);
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);*/
    Drivetrain drive;// = new Drivetrain(apriltagCamera);
    // Claw claw = new Claw();

    private final Timer autoTimer15Sec = new Timer();

    //PIDController turnControllerImu = new PIDController(0.1, 0, 0);
    AHRS ahrs; // altitude heading reference system aka navigator

    private double arcadeDriveCommand = 0.0;
    private double arcadeSteerCommand = 0.0;

    private double tankDriveLeftSpeed = 0.0;
    private double tankDriveRightSpeed = 0.0;

    private static ErrorCode error; // CTRE specific error codes

    private enum Mission {
        INIT, // stay at A; reset; do some testings: arm, lights, etc.
        MOVE_A2B, // move 45 degrees forward 3 feet
        CROSS_DOCK_B2C, // 0 degree to cross the charging station
        SEEK_OBJ_C, // vision, seek the cone/cube, and pick it up
        MOVE_C2D,// trun 90 degree, 4 feet
        SEEK_TAG_D, // look forward to seek apriltag, drop the cone/cube
        MOVE_STAY_D2E, // backward to charger station, turn -90 degree, and move backward on to station
        EXIT
    }
    private enum GameElement {
        CUBE,
        CONE,
        TAPE
    }

    public StateMachine() {
        drive = RobotContainer.drive;
        apriltagCamera = RobotContainer.vision;
        objDetectCamera = RobotContainer.objDetectCamera;
        ahrs = RobotContainer.ahrs.ahrs;
        /*if ((error = leftMotor.configFactoryDefault()) != ErrorCode.OK) // factory default the motors
            System.out.println("error setting leftMotor to defaults = " + error);
        if ((error = rightMotor.configFactoryDefault()) != ErrorCode.OK)
            System.out.println("error setting rightMotor to defaults = " + error);
        if ((error = leftFollower.configFactoryDefault()) != ErrorCode.OK)
            System.out.println("error setting leftFollower to defaults = " + error);
        if ((error = rightFollower.configFactoryDefault()) != ErrorCode.OK)
            System.out.println("error setting rightFollower to defaults = " + error);

        leftFollower.follow(leftMotor); // attach followers to leaders
        rightFollower.follow(rightMotor);
        leftMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
        rightMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
        leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.setInverted(InvertType.FollowMaster);
        leftMotor.setSensorPhase(true); // <<<<<< Adjust this
        rightMotor.setSensorPhase(false); // <<<<<< Adjust this*/



        turnControllerImu.enableContinuousInput(-180.0f, 180.0f); // set input range from navigator
        turnControllerImu.setIntegratorRange(-1.0, 1.0); // PID output range to correct
        turnControllerImu.setTolerance(1.0); // tolerance around set heading to accept
        
        ahrs.zeroYaw();
    }

    int missionStepTimeout = 10; // TODO debug
    private Mission currentMission = Mission.INIT; // debug: TODO
    private Mission previousMission = Mission.INIT;
    private final Timer missionRunTimer = new Timer();

    private void setMissionTo(Mission newMission) {
        // debug: TODO
        if (newMission == Mission.CROSS_DOCK_B2C) {
            currentMission = Mission.EXIT;
            missionStepTimeout = 1000;
            return;
        }
        // end debug

        previousMission = currentMission;
        currentMission = newMission;
        missionRunTimer.reset();
        setTaskTo(0);
    }

    private int currentTaskID = 0;
    private int previousTaskID = 0;
    private final Timer taskRunTimeout = new Timer();

    private void setTaskTo(int newTaskID) {
        previousTaskID = currentTaskID;
        currentTaskID = newTaskID;
        taskRunTimeout.reset();
        taskRunTimeout.start();
    }

    public void ExecutionPeriod() {
        switch (currentMission) {
            case INIT:
                // find self init position, read parking zone picture, drop preload
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionINIT();
                break;
            case MOVE_A2B:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionMOVE_A2B();
                break;
            case CROSS_DOCK_B2C:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionCROSS_DOCK_B2C();
                break;
            case SEEK_OBJ_C:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionSEEK_OBJ_C(GameElement.CONE);
                break;
            case MOVE_C2D:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionMOVE_C2D();
                break;   
            case SEEK_TAG_D:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionSEEK_TAG_D();
                break; 
            case MOVE_STAY_D2E:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionMOVE_STAY_D2E();
                break; 
            default: // EXIT
                break;
        }

        // motorsRunnable();

        // addLog();

        // manualTuningMotors(gamepad1);

    }

    public void report()
    {
        //smartdashboard
        systemReport();
        drivebaseReport();
        armReport();
        imuReport();
        apriltagReport();
        objDetectionReport();
    }

    private void missionINIT() {
        if (currentTaskID == 0) {
            turnControllerImu.setSetpoint(ahrs.getYaw());
            resetDriveLoops();
            setTaskTo(1);
        } else if (currentTaskID == 1) {
            if (taskRunTimeout.get() > 3) {
                setTaskTo(2);
            }
        } else {
            setMissionTo(Mission.MOVE_A2B);
        }
    }

    private void missionMOVE_A2B() {
        if (currentTaskID == 0) {
            // set encoder to zero
            /*if ((error = leftMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {
                System.out.println("error setting sensor position to 0 in auto init");
            }*/
            setTaskTo(1);
        } else if (currentTaskID == 1) {
            boolean done = driveStraightLoop(0.5, 36, 45, 0, true);
            if(taskRunTimeout.get() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setMissionTo(Mission.CROSS_DOCK_B2C);
            }
            else{}
        } 
    }

    private void missionCROSS_DOCK_B2C()
    {
        if (currentTaskID == 0) {
            boolean done = driveStraightLoop(0.8, 24, 0, 1, true);
            if (taskRunTimeout.get() >= 2) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(1);
            }
        } else if (currentTaskID == 1) {
            boolean done = driveStraightLoop(0.3, 24, 0, -1, true);
            if (taskRunTimeout.get() >= 2) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
        } else {
            setMissionTo(Mission.SEEK_OBJ_C);
        }
    }

    private void missionSEEK_OBJ_C(GameElement type)
    {
        if (currentTaskID == 0) {
            boolean hasValidTarget = Update_Limelight_Tracking(type);
            if (taskRunTimeout.get() >= 5) {
                // timeout, bad! we might to move robot a bit to try it again
                setMissionTo(Mission.EXIT);
            }
            else if (hasValidTarget)
            {
                drive.arcadeDrive(arcadeDriveCommand,arcadeSteerCommand);
                if(NerdyMath.inRange(arcadeDriveCommand, -0.05, 0.05) &&
                NerdyMath.inRange(arcadeSteerCommand, -0.05, 0.05))
                {
                    setTaskTo(1);
                }
            }
        } else if (currentTaskID == 1) {
            if(type == GameElement.CONE || type == GameElement.CUBE)
            {
                // close the claw
                clawControl(false);
            }
            else // GameElement.TAPE
            {
                // open the claw
                clawControl(true);
            }
            setTaskTo(2);
        } else {
            if (taskRunTimeout.get() > 1) {
                setMissionTo(Mission.MOVE_C2D);
            }
        }
    }

    private void missionMOVE_C2D()
    {
        if (currentTaskID == 0) {
            /*if ((error = leftMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {// set encoder to zero
                System.out.println("error setting sensor position to 0 in auto init");
            }*/
            setTaskTo(1);
        } else if (currentTaskID == 1) {
            boolean done = driveStraightLoop(0.5, 48, -90, 0, false);
            if(taskRunTimeout.get() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setMissionTo(Mission.SEEK_TAG_D);
            }
            else{}
        } 
    }

    private void missionSEEK_TAG_D()
    {
        if (currentTaskID == 0) {
            boolean hasValidTarget = updateApriltagTracking(22);
            if (taskRunTimeout.get() >= 5) {
                // timeout, bad! we might to move robot a bit to try it again
                setMissionTo(Mission.EXIT);
            }
            else if (hasValidTarget)
            {
                if(NerdyMath.inRange(arcadeDriveCommand, -0.05, 0.05) &&
                NerdyMath.inRange(arcadeSteerCommand, -0.05, 0.05))
                {
                    drive.arcadeDrive(0,0);
                    /*if ((error = leftMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {// set encoder to zero
                        System.out.println("error setting sensor position to 0 in auto init");
                    }*/
                    setTaskTo(1);
                }
                else{
                    drive.arcadeDrive(arcadeDriveCommand,arcadeSteerCommand);
                }
            }
        } else if (currentTaskID == 1) {
            boolean done = driveStraightLoop(0.3, 5, 0, 0, false);
            if(taskRunTimeout.get() >= 3)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
            else{}
        } else if (currentTaskID == 2) {
            // open the claw
            clawControl(true);
            setTaskTo(3);
        }
        else {
            if (taskRunTimeout.get() > 1) {
                setMissionTo(Mission.MOVE_STAY_D2E);
            }
        }
    }

    private void missionMOVE_STAY_D2E()
    {
        if (currentTaskID == 0) {
            
        } else if (currentTaskID == 1) {
            if (taskRunTimeout.get() > 3) {
                setTaskTo(2);
            }
        } else {
            setMissionTo(Mission.EXIT);
        }
    }

    /*
     * ====================================================================================================
     * Vision functions are below this line.
     * Finding the object/apriltag 
     * ====================================================================================================
     */
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    private boolean Update_Limelight_Tracking( GameElement gameObj )
    {
        // These numbers must be tuned for your Robot!  Be careful!
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        boolean m_LimelightHasValidTarget = objDetectCamera.hasValidTarget();// tableApriltag.getEntry("tv").getDouble(0);
        double tx = objDetectCamera.getHorizontalLength();//tableApriltag.getEntry("tx").getDouble(0);
        double ty = objDetectCamera.getVerticalLength();//tableApriltag.getEntry("ty").getDouble(0);
        double ta = objDetectCamera.getArea();//tableApriltag.getEntry("ta").getDouble(0);

        if (m_LimelightHasValidTarget == false)
        {
          arcadeDriveCommand = 0.0;
          arcadeSteerCommand = 0.0;
          return false;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        arcadeSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        arcadeDriveCommand = drive_cmd;

        return true;
    }

    PIDController forwardControllerApriltag = new PIDController(0.1, 0, 0);
    PIDController turnControllerApriltag = new PIDController(0.1, 0, 0);
    private boolean updateApriltagTracking(int tagID) {
        //double forwardSpeed;
        //double rotationSpeed;

        //forwardSpeed = -xboxController.getRightY();

        //if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision

            /*if (apriltagCamera.limelightHasTargets) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                arcadeSteerCommand = -turnControllerApriltag.calculate(apriltagCamera.getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                arcadeSteerCommand = 0;
            }*/
            if (apriltagCamera.limelightHasTargets) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(apriltagCamera.getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                arcadeDriveCommand = -forwardControllerApriltag.calculate(range, GOAL_RANGE_METERS);

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                arcadeSteerCommand = -turnControllerApriltag.calculate(apriltagCamera.getYaw(), 0);
            } else {
                // If we have no targets, stay still.
            }
        /* } else {
            // Manual Driver Mode
            rotationSpeed = xboxController.getLeftX();
        }*/
        return apriltagCamera.limelightHasTargets;
    }

    /*
     * ====================================================================================================
     * All Motor functions are below this line.
     * ====================================================================================================
     */

    boolean clawStatusOpen = false;
    private void clawControl( boolean doOpen) {
        clawStatusOpen = doOpen;
        if(doOpen) {
            RobotContainer.claw.clawOpen().schedule();
        }
        else {
            RobotContainer.claw.clawClose().schedule();
        }
    }

    final double kOffBalanceAngleThresholdDegrees = 10;
    final double kOonBalanceAngleThresholdDegrees  = 5;
    boolean autoBalanceXMode;
    boolean autoBalanceYMode;
    // todo, replaced by a PID controller in order to provide a tuning mechanism appropriate to the robot
    private void balancingControl() {
    //while (isOperatorControl() && isEnabled()) 
    {
        //double xAxisRate            = stick.getX();
        //double yAxisRate            = stick.getY();
        double pitchAngleDegrees    = ahrs.getPitch();
        double rollAngleDegrees     = ahrs.getRoll();
        if ( !autoBalanceXMode && 
                    (Math.abs(pitchAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode && 
                        (Math.abs(pitchAngleDegrees) <= 
        Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if ( !autoBalanceYMode && 
                    (Math.abs(pitchAngleDegrees) >= 
        Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
        }
        else if ( autoBalanceYMode && 
                        (Math.abs(pitchAngleDegrees) <= 
        Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
        }
        // Control drive system automatically, 
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle
        if ( autoBalanceXMode ) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            //xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
        if ( autoBalanceYMode ) {
            double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
            //yAxisRate = Math.sin(rollAngleRadians) * -1;
        }
        //myRobot.mecanumDrive_Cartesian(xAxisRate, yAxisRate, stick.getTwist(),0);
        Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    private double inches2Ticks(double inches) {
        return ((inches * encoderTicksPerRotation) / (wheelDiameter * Math.PI));
    }

    private void resetDriveLoops()
    {
        hasInitStraight = false;
        hasInitTurnTo = false;
        hasInitHold = false;
        hasInitStrafe = false;
    }

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    *  maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    *  distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    *  heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    private boolean hasInitStraight = false;
    //PIDController forwardControllerImu = new PIDController(0.1, 0, 0);
    PIDController turnControllerImu = new PIDController(0.1, 0, 0);

    private double ticks2Go; // how many encode ticks to move
    private double ticks2SlowDown; // when to slow so you don't overshoot

    private boolean driveStraightLoop(double maxDriveSpeed,
                                     double distance,
                                     double heading,
                                     int upOrDown,
                                     boolean continueMove) {
        if(!hasInitStraight) {
            hasInitStraight = true;
            drivePower = maxDriveSpeed;
            ticks2Go = inches2Ticks(distance); // set up encoder stop condition
            ticks2SlowDown = inches2Ticks(distance*0.2); // set up encoder slow down condition
        }

        double position = 0;//leftMotor.getSelectedSensorPosition(0);
        if ((ticks2Go + position) < ticks2SlowDown)
            drivePower = 0.3; // cut power prepare to stop

        if (position <= -ticks2Go) { // reached desired encoder position
            // if !continueMove then tankDrive (0,0)
            return true;
        } 
        else { // move straight
            double rotateToAngleRate = turnControllerImu.calculate(ahrs.getYaw(), heading); // calc error correction
            tankDriveLeftSpeed = -(drivePower + rotateToAngleRate);
            tankDriveRightSpeed = -(drivePower - rotateToAngleRate);
            drive.tankDrive(tankDriveLeftSpeed, tankDriveRightSpeed);
                                                                                                    
            // motor powers
            //System.out.println("drive power = " + drivePower + "  rotToAngleRate = " + rotateToAngleRate
            //        + "  sensorPosition = " + leftMotor.getSelectedSensorPosition(0) + "  ticksToGo = " + ticks2Go);
            return false;
        } 
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     *  maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     *  heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    private boolean hasInitTurnTo = false;
    private boolean turnToHeadingLoop(double maxTurnSpeed, double heading)
    {
        if(!hasInitTurnTo)
        {
            double turningSpeed = turnControllerImu.calculate(drive.getHeading(), heading);

            if(NerdyMath.inRange(arcadeSteerCommand, 0 , maxTurnSpeed)) {
                drive.tankDrive(turningSpeed, turningSpeed * -1);
            }
        }
        return true;
    }



    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     *  maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     *  heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     *  holdTime   Length of time (in seconds) to hold the specified heading.
     */
    private boolean hasInitHold = false;
    //ElapsedTime holdTimer = new ElapsedTime();
    private boolean holdHeadingLoop(double maxTurnSpeed, double heading, double holdTime)
    {
        if(!hasInitHold)
        {
            hasInitHold = true;
            //holdTimer.reset();
        }
        return true;
    }

    private boolean hasInitStrafe = false;
    //ElapsedTime runtimeStrafe = new ElapsedTime();
    // Positive Inch: move left.
    private boolean driveStrafeLoop(double Inches, double maxSpeed, int timeoutInSeconds, double heading)
    {
        if(!hasInitStrafe)
        {
            hasInitStrafe = true;
        }
        return true;
    }

    /*
     * ====================================================================================================
     * Data Report functions are below this line.
     * ====================================================================================================
     */
    private void imuReport()
    {
       /*boolean zero_yaw_pressed = stick.getTrigger();
        if ( zero_yaw_pressed ) {
            ahrs.zeroYaw();
        }*/

        /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        /* 
        // Display tilt-corrected, Magnetometer-based heading (requires             
        // magnetometer calibration to be useful)                                   
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
        
        // Display 9-axis Heading (requires magnetometer calibration to be useful)  
        SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        // These functions are compatible w/the WPI Gyro Class, providing a simple  
        // path for upgrading from the Kit-of-Parts gyro to the navx-MXP            
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        // Display Processed Acceleration Data (Linear Acceleration, Motion Detect) 
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

        // Display estimates of velocity/displacement.  Note that these values are  
        // not expected to be accurate enough for estimating robot position on a    
        // FIRST FRC Robotics Field, due to accelerometer noise and the compounding 
        // of these errors due to single (velocity) integration and especially      
        // double (displacement) integration.                                       
        
        SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
        
        // Display Raw Gyro/Accelerometer/Magnetometer Values                       
        // NOTE:  These values are not normally necessary, but are made available   
        // for advanced users.  Before using this data, please consider whether     
        // the processed data (see above) will suit your needs.                     
        
        SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        
        // Omnimount Yaw Axis Information                                           
        // For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        // Sensor Board Information                                                 
        SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        // Quaternion Data                                                          
        // Quaternions are fascinating, and are the most compact representation of  
        // orientation data.  All of the Yaw, Pitch and Roll Values can be derived  
        // from the Quaternions.  If interested in motion processing, knowledge of  
        // Quaternions is highly recommended.                                       
        SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
        
        */
        // Connectivity Debugging Support                                           
        SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
    }

    private void apriltagReport() {
        SmartDashboard.putBoolean("HasTarget", apriltagCamera.limelightHasTargets);
        if(apriltagCamera.limelightHasTargets)
        {
            SmartDashboard.putNumber("Horizontal Offset", apriltagCamera.getYaw());
            SmartDashboard.putNumber("Vertical Offset", apriltagCamera.getPitch());
            //SmartDashboard.putNumber("Area", apriltagCamera.getArea());
            SmartDashboard.putNumber("Tag ID", apriltagCamera.getFiducialId());
        }
    }

    private void objDetectionReport() {
        SmartDashboard.putBoolean("HasTarget", objDetectCamera.hasValidTarget());
        SmartDashboard.putNumber("Horizontal Offset", objDetectCamera.getXAngle());
        SmartDashboard.putNumber("Vertical Offset", objDetectCamera.getYAngle());
        SmartDashboard.putNumber("Area", objDetectCamera.getArea());
        SmartDashboard.putNumber("Skew", objDetectCamera.getSkew());
    }

    private void systemReport() {
        SmartDashboard.putString("Mission ID", currentMission.toString());
        SmartDashboard.putNumber("Task ID", currentTaskID);
    }

    private void drivebaseReport()
    {
        SmartDashboard.putNumber("Tank Drive Left", tankDriveLeftSpeed);
        SmartDashboard.putNumber("Tank Drive Right" ,tankDriveRightSpeed);
        SmartDashboard.putNumber("Arcade Drive", arcadeDriveCommand);
        SmartDashboard.putNumber("Arcade Steer" ,arcadeSteerCommand);
    }

    private void armReport()
    {
        SmartDashboard.putBoolean("Claw Open", clawStatusOpen);
    }
}