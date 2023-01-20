package frc.robot.states;

import java.sql.Struct;
import java.util.ResourceBundle.Control;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DataLogManager;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Limelight.LightMode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.util.NerdyMath;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
        SEEK_TAG, // look forward to seek apriltag, drop the cone/cube
        SEEK_OBJ_DROP_CONE, // vision, seek the tape, and drop cone
        SEEK_OBJ_PICK_CONE, // vision, seek the cone, and pick it up
        MOVE_C2D,// trun 90 degree, 4 feet
        CROSS_DOCK, // 0 degree to cross the charging station
        STAY_DOCK, // backward to charger station, turn -90 degree, and move backward on to station
        EXIT
    }

    public enum GameElement {
        // make sure sync with Camera hardware configuration
        NONE(0), CONE(1), CUBE(2),  TAPE(3);

        private int type;

        private GameElement(int type) {
            this.type = type;
        }
        public int getType() {
            return type;
        }
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
        
        //ahrs.zeroYaw();
        ahrs.reset();// Clockwise = positive angle
    }

    private final int missionStepTimeout = 1000; // TODO debug
    private Mission currentMission = Mission.INIT; 
    private final Timer missionRunTimer = new Timer();
    private void setMissionTo(Mission newMission) {
        currentMission = newMission;
        setTaskTo(0);
        missionRunTimer.reset();
        missionRunTimer.start();
    }

    private int currentTaskID = 0;
    private final Timer taskRunTimeout = new Timer();
    private void setTaskTo(int newTaskID) {
        currentTaskID = newTaskID;
        taskRunTimeout.reset();
        taskRunTimeout.start();
    }

    public void ReinitExecution()
    {
        setMissionTo(Mission.INIT);
    }

    private int MissionHeartBeat = 0;
    public void ExecutionPeriod() {
        MissionHeartBeat++;
        switch (currentMission) {
            case INIT:
                // find self init position, read parking zone picture, drop preload
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionINIT(Mission.MOVE_A2B);
                break;
            case MOVE_A2B:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionMOVE_A2B(Mission.SEEK_OBJ_PICK_CONE);
                break; 
            case SEEK_OBJ_PICK_CONE:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionSEEK_OBJ(GameElement.CONE, Mission.MOVE_C2D);
                break;
            case SEEK_TAG:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionSEEK_TAG();
                break; 
            case CROSS_DOCK:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionCROSS_DOCK(Mission.EXIT);
                break;
            case SEEK_OBJ_DROP_CONE:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionSEEK_OBJ(GameElement.TAPE, Mission.STAY_DOCK);
                break;
            case MOVE_C2D:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionMOVE_C2D(Mission.SEEK_OBJ_DROP_CONE);
                break;  
            case STAY_DOCK:
                if (missionRunTimer.get() > missionStepTimeout) {
                    // Error!
                    setMissionTo(Mission.EXIT);
                    break;
                }
                missionSTAY_DOCK();
                break; 
            default: // EXIT
                missionEXIT(false);
                break;
        }

        // motorsRunnable();
        // addLog();

        if(currentMission == Mission.EXIT)
            manualTuningMotors(RobotContainer.operatorController);
    }

    private static final double kTrackWidth = 0.381 * 2; // meters
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
    private void manualTuningMotors(CommandPS4Controller controller)
    {
        if( controller == null) return;

        double forward = -1.0 * controller.getLeftY();	// Sign this so forward is positive
		double turn = controller.getRightX();       // Sign this so right is positive
        
        /* Deadband - within 10% joystick, make it zero */
		if (Math.abs(forward) < 0.10) {
			forward = 0;
		}
		if (Math.abs(turn) < 0.10) {
			turn = 0;
		}
        // TODO test: let turn=0, to see if robot is able to go Straight without gyro
        //https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java%20Talon%20FX%20(Falcon%20500)
        if( Math.abs(forward) >= 0.10 || Math.abs(turn) > 0.10 ) {
            drive.arcadeDiffDrive(forward, turn);
        }

        //https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdrivebot/Drivetrain.java
        /*final var xSpeed = -m_speedLimiter.calculate(forward) * kMaxSpeed;
        final var rot = -m_rotLimiter.calculate(turn) * kMaxAngularSpeed;

        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));

        final double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

        final double leftOutput =
            m_leftPIDController.calculate(m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
        final double rightOutput =
            m_rightPIDController.calculate(m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);
        //m_leftGroup.setVoltage();
        //m_rightGroup.setVoltage();
        drive.setPower(leftOutput + leftFeedforward, rightOutput + rightFeedforward);*/
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

    public void logdata()
    {
        // Record both DS control and joystick data
        //systemLog();
        //drivebaseLog);
        //armLog();
        //imuLog();
        //apriltagLog();
        //objDetectionLog();
    }

    private void missionINIT(Mission nextMission) {
        if (currentTaskID == 0) {
            //turnControllerImu.setSetpoint(ahrs.getYaw());
            resetDriveLoops();
            ahrs.zeroYaw();
            //ahrs.reset();// Clockwise = positive angle
            setTaskTo(1);
        } else if (currentTaskID == 1) {
            if (taskRunTimeout.get() > 2) {
                setTaskTo(2);
            }
        } else {
            setMissionTo(nextMission);
        }
    }

    private void missionMOVE_A2B(Mission nextMission) {
        if (currentTaskID == 0) {
            // set encoder to zero
            /*if ((error = leftMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {
                System.out.println("error setting sensor position to 0 in auto init");
            }*/
            setTaskTo(1);
        } else if (currentTaskID == 1) {
            boolean done = driveStraightLoop(0.5, 2, 45, 0, false);
            if(taskRunTimeout.get() >= 10)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(nextMission);// TODO DEBUG Mission.EXIT);
            }
            else if( done )
            {
                setMissionTo(nextMission);
            }
            else{}
        } 
    }

    private void missionCROSS_DOCK(Mission nexMission)
    {
        if (currentTaskID == 0) {
            boolean done = driveUpLoop(0.8, 1, 0, 1, false);
            if (taskRunTimeout.get() >= 3) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                //setTaskTo(1); TODO DEBUG
                setMissionTo(Mission.EXIT);
            }
        } 
        /*else if (currentTaskID == 1) { // 20ms momentum 
            boolean done = driveHoldLoop(0.3, 1, 0, -1, true);
            if (taskRunTimeout.get() >= 2) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setTaskTo(2);
            }
        }

        if (currentTaskID == 2) {
            boolean done = driveDownLoop(0.3, 1, 0, -1, true);
            if (taskRunTimeout.get() >= 2) {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                setMissionTo(nexMission);
            }
        } */
    }

    CAMERA_MODE objCameraStatus = CAMERA_MODE.IDLE;
    private void missionSEEK_OBJ(GameElement type, Mission nexMission)
    {
        if (currentTaskID == 0) {
            resetObjDetectionLoops();
            objCameraStatus = CAMERA_MODE.IDLE;
            setTaskTo(1);
        }
        else if(currentTaskID == 1) {
            objCameraStatus = UpdateObjectTracking(type);
            if (taskRunTimeout.get() >= 5000) { // TODO DEBUG
                // timeout, bad! we might move robot a bit and try again
                resetObjDetectionLoops();
                setMissionTo(Mission.EXIT);
            }
            else if (objCameraStatus == CAMERA_MODE.ARRIVED)
            {
                setTaskTo(2);
            }
        } else if (currentTaskID == 2) {
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
            setTaskTo(3);
        } else {
            if (taskRunTimeout.get() > 1) { 
                setMissionTo(nexMission);
            }
        }
    }

    private void missionMOVE_C2D(Mission nextMission)
    {
        if (currentTaskID == 0) {
            /*if ((error = leftMotor.setSelectedSensorPosition(0.0)) != ErrorCode.OK) {// set encoder to zero
                System.out.println("error setting sensor position to 0 in auto init");
            }*/
            setTaskTo(1);
        } else if (currentTaskID == 1) {
            boolean done = driveStraightLoop(0.5, 4, -90, 0, false);
            if(taskRunTimeout.get() >= 5)
            {
                // timeout, bad! should not happen at all
                resetDriveLoops();
                setMissionTo(nextMission);// TODO debug Mission.EXIT);
            }
            else if( done )
            {
                setMissionTo(nextMission);
            }
            else{}
        } 
    }

    private void missionSEEK_TAG()
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
                setMissionTo(Mission.STAY_DOCK);
            }
        }
    }

    private void missionSTAY_DOCK()
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

    private void missionEXIT(boolean doStop)
    {
        if(doStop) {
            drive.setPower(0, 0);
        }
    }

    /*
     * ====================================================================================================
     * Vision functions are below this line.
     * Finding the object/apriltag 
     * ====================================================================================================
     */

     private void resetObjDetectionLoops()
     {
        hadInitGameObjDetection = false;
     }

    private boolean hadInitGameObjDetection = false;
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    enum CAMERA_MODE
    {
        WAIT, // found zero, or more than 1
        IDLE, // doing nothing, wrong pipeline or others
        STOP, // stop movement
        ACTION,// detected one, and approach to it
        ARRIVED // found
    }
    double driveMaxPowerObjDetection = 0.6;
    double driveMinPowerToMove = 0.1;
    PIDController forwardControllerObj = new PIDController(0, 0, 0);
    PIDController turnControllerObj = new PIDController(0, 0, 0);
    /*class ObjectDistance2Area
    {
        double TARGET_REAL_SIZE = 1;
        double DESIRED_TARGET_AREA = 1; // Area of the target, with 1 meter distance
        double TARGET_1dot5METER_AREA = 1;
        double TARGET_2_METERS_AREA = 1;
    }
    ObjectDistance2Area objectConf = new ObjectDistance2Area();*/
    //ProfiledPIDController a; // TODO
    private CAMERA_MODE UpdateObjectTracking( GameElement gameObj )
    {
        // These numbers must be tuned for your Robot!  Be careful!
        // 0.20->1.5 meters; 0.104->2 meters (object size: 5x10 cm2)
             
        final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast
        if(!hadInitGameObjDetection)
        {
            hadInitGameObjDetection = true;
            objDetectCamera.setPipeline(gameObj.getType());
            if(gameObj == GameElement.CONE)
            {
                forwardControllerObj.setP(0.05);
                turnControllerObj.setP(0.03);
            }
            else if(gameObj == GameElement.CUBE)
            {
                forwardControllerObj.setP(0.1);
                turnControllerObj.setP(0.1);
            }
            else if(gameObj == GameElement.TAPE)
            {
                objDetectCamera.setLightState(LightMode.ON);
                forwardControllerObj.setP(0.08);
                turnControllerObj.setP(0.03);
                /*objectConf.DESIRED_TARGET_AREA = 0.437; 
                objectConf.TARGET_1dot5METER_AREA = 0.2;
                objectConf.TARGET_2_METERS_AREA = 0.104;
                objectConf.TARGET_REAL_SIZE = 0.05*0.1;*/
            }
            else {
                hadInitGameObjDetection = false;
                objDetectCamera.setLightState(LightMode.OFF);
                return CAMERA_MODE.IDLE;
            }
        }

        boolean hasValidTarget = objDetectCamera.hasValidTarget();// tableApriltag.getEntry("tv").getDouble(0);
        // calibration is needed
        double tx = objDetectCamera.getXAngle();//getHorizontalLength();//tableApriltag.getEntry("tx").getDouble(0);
        double ty = objDetectCamera.getYAngle();//getVerticalLength();//tableApriltag.getEntry("ty").getDouble(0);
        double ta = objDetectCamera.getArea();//tableApriltag.getEntry("ta").getDouble(0);

        
        //double leftpower;
        //double rightpower;

        if (hasValidTarget == false)
        {
            tankDriveLeftSpeed = tankDriveRightSpeed = 0.0;
            drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);
            return CAMERA_MODE.WAIT;
        }

        double steering_adjust = 0;
        if(tx > 1.0)
        {
            steering_adjust = (tx*turnControllerObj.getP()) + driveMinPowerToMove;
        }
        else if(tx < -1.0)
        {
            steering_adjust = (tx*turnControllerObj.getP()) - driveMinPowerToMove;
        }

        double distance_adjust = -1 * ty * forwardControllerObj.getP();
        
        tankDriveLeftSpeed = distance_adjust + steering_adjust;
        tankDriveRightSpeed = distance_adjust - steering_adjust;
        //drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);

        /* 
        if (ta >= objectConf.DESIRED_TARGET_AREA) { // reached desired encoder position
                leftpower = 0;
                rightpower = 0;
        } 
        else {// move straight
            double a = forwardControllerObj.calculate(ta, objectConf.DESIRED_TARGET_AREA);
            if(a > 0 && a < driveMinPowerToMove) {
                a = driveMinPowerToMove;
            }
            else if( a < 0 && a > -1*driveMinPowerToMove)
            {
                a = -1*driveMinPowerToMove;
            }
            leftpower = rightpower = a;
        }

        double turncmd = turnControllerObj.calculate(tx, 0); // TODO, calibration is needed
        leftpower += turncmd;
        rightpower -= turncmd;*/
        tankDriveLeftSpeed = NerdyMath.clamp(tankDriveLeftSpeed, -driveMaxPowerObjDetection, driveMaxPowerObjDetection);
        tankDriveRightSpeed = NerdyMath.clamp(tankDriveRightSpeed, -driveMaxPowerObjDetection, driveMaxPowerObjDetection);
        
        if(NerdyMath.inRangeLess(tankDriveLeftSpeed, -1*driveMinPowerToMove, driveMinPowerToMove) && 
            NerdyMath.inRangeLess(tankDriveRightSpeed, -1*driveMinPowerToMove, driveMinPowerToMove) ) {

            tankDriveLeftSpeed = tankDriveRightSpeed = 0;
            drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);
        
            hadInitGameObjDetection = false;
            objDetectCamera.setLightState(LightMode.OFF);
            return CAMERA_MODE.ARRIVED;
        }
        else {
            //tankDriveLeftSpeed = leftpower;
            //tankDriveRightSpeed = rightpower;
            drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);
            return CAMERA_MODE.ACTION;
        }
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
            RobotContainer.claw.clawPiston.set(Value.kForward);
        }
        else {
            RobotContainer.claw.clawPiston.set(Value.kReverse);
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
        hasInitDriveUp = false;
    }

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if Move gets to the desired position
    *
    *  maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    *  distance   Distance to move from current position.  Negative distance means move backward.
    *  heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    private boolean hasInitStraight = false;
    //PIDController forwardControllerImu = new PIDController(0.1, 0, 0);
    PIDController turnControllerImu = new PIDController(0.001, 0, 0);

    private double ticks2Go; // how many encode ticks to move
    private double ticks2SlowDown; // when to slow so you don't overshoot

    private boolean driveStraightLoop(double maxForwardDriveSpeed,
                                     double distanceMeter,
                                     double heading,
                                     int upOrDown,
                                     boolean continueMove) {
        if(!hasInitStraight) {
            drive.resetEncoder();
            hasInitStraight = true;
            drivePower = 0.3;
            ticks2Go = drive.meterToTicks(distanceMeter);//inches2Ticks(distance); // set up encoder stop condition
            ticks2SlowDown = ticks2Go*0.8;//inches2Ticks(distance*0.2); // set up encoder slow down condition
        }

        double position = drive.getTicks();
        if (position >= ticks2SlowDown)
            drivePower = 0.15; // cut power prepare to stop

        if (position >= ticks2Go) { // reached desired encoder position
            if (continueMove) {
                drive.setPower(0.1,0.1);
            } 
            else {
                drive.setPower(0, 0);
            }
            return true;
        } 
        else { // move straight
            double rotateToAngleRate = turnControllerImu.calculate(ahrs.getYaw(), heading); // calc error correction
            //tankDriveLeftSpeed = (drivePower + rotateToAngleRate);
            //tankDriveRightSpeed = (drivePower - rotateToAngleRate);
            tankDriveLeftSpeed = NerdyMath.clamp((drivePower + rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
            tankDriveRightSpeed = NerdyMath.clamp((drivePower - rotateToAngleRate), -maxForwardDriveSpeed, maxForwardDriveSpeed);
            drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);

            // motor powers
            //System.out.println("drive power = " + drivePower + "  rotToAngleRate = " + rotateToAngleRate
            //        + "  sensorPosition = " + leftMotor.getSelectedSensorPosition(0) + "  ticksToGo = " + ticks2Go);
            return false;
        } 
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     */
    private boolean hasInitTurnTo = false;
    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(1, 0.7);
    private final ProfiledPIDController turnControllerProfiledImu =
        new ProfiledPIDController(0.05, 0.0, 0.1, m_constraints, 0.02);
    double[] turnLog = new double[3];
    private boolean turnToAngleLoop(double heading) // heading -179 to +179
    {
        double turningSpeed = turnControllerProfiledImu.calculate(ahrs.getYaw(), heading);
        turningSpeed = NerdyMath.clamp(turningSpeed, -0.7, 0.7);
        drive.tankDrive(turningSpeed, -1*turningSpeed);
        turnLog[0] = turnControllerProfiledImu.getPositionError();
        turnLog[1] = turningSpeed;
        turnLog[2] = ahrs.getYaw();
        if(NerdyMath.inRangeLess(ahrs.getYaw(), heading-1, heading+1)) {
            return true;
        }
        else{
            return false;
        }
    }

    boolean hasInitDriveUp = false;
    boolean OnSlope = false;
    int driveUpStatus = 0;
    PIDController driveUpForwardControllerImu = new PIDController(0.1, 0, 0);
    PIDController driveUpturnControllerImu = new PIDController(0.001, 0, 0);
    private boolean driveUpLoop(double maxForwardDriveSpeed,
                                double distanceMeter,
                                double heading,
                                int upOrDown,
                                boolean continueMove) {
        if(!hasInitDriveUp) {
            hasInitDriveUp = true;
            drive.resetEncoder();
            ticks2Go = drive.meterToTicks(distanceMeter);
            ticks2SlowDown = ticks2Go*0.8;
            drivePower = 0.3;
            OnSlope = false;
            driveUpStatus = 0;
        }

        if(!OnSlope){
            if(ahrs.getPitch() > 10) {
                OnSlope = true;
            }
        }

        double position = drive.getTicks();
        if (position >= ticks2SlowDown)
            drivePower = 0.15; // cut power prepare to stop

        boolean finished = false;
        if (position >= ticks2Go && OnSlope ) { // reached desired encoder position
            if( NerdyMath.inRange(ahrs.getYaw(), -10, 10)) {
                if( ahrs.getPitch() < 30 ) {
                    if (continueMove) {
                        drive.setPower(0.1,0.1);
                    } 
                    else {
                        drive.setPower(0, 0);
                    }
                    finished = true;
                } 
                else {
                    finished = false; // need to add more forward power with shorter distance
                    driveUpStatus = 1;
                }
            }
            else {
                if( ahrs.getPitch() > 30 ) {
                    finished = false; // need to add more forward and turn power with longer distance, should no wheel moves backward
                    driveUpStatus = 2;
                }
                else {
                    finished = false; // need to add more turn power without distance, should no wheel moves backward
                    driveUpStatus = 3;
                }
            }
        } 

        if(!finished) { // move straight
            double rotateToAngleRate = driveUpturnControllerImu.calculate(ahrs.getYaw(), heading); // calc error correction
            double driveUpPower = driveUpForwardControllerImu.calculate(ahrs.getPitch(), 30);
            tankDriveLeftSpeed = NerdyMath.clamp((drivePower + driveUpPower + rotateToAngleRate), 0, maxForwardDriveSpeed);
            tankDriveRightSpeed = NerdyMath.clamp((drivePower + driveUpPower - rotateToAngleRate), 0, maxForwardDriveSpeed);
            drive.setPower(tankDriveLeftSpeed, tankDriveRightSpeed);

            return false;
        } 
        else {
            return true;
        }
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
        SmartDashboard.putString("Obj Camera Status", objCameraStatus.toString());
        SmartDashboard.putBoolean("Obj HasTarget", objDetectCamera.hasValidTarget());
        SmartDashboard.putNumber("Obj Horizontal Offset", objDetectCamera.getXAngle());
        SmartDashboard.putNumber("Obj Vertical Offset", objDetectCamera.getYAngle());
        SmartDashboard.putNumber("Obj Area", objDetectCamera.getArea());
        SmartDashboard.putNumber("Obj Skew", objDetectCamera.getSkew());
    }

    private void systemReport() {
        SmartDashboard.putString("Mission ID", currentMission.toString());
        SmartDashboard.putNumber("Mission Task ID", currentTaskID);
        SmartDashboard.putNumber("Mission Running", MissionHeartBeat);
        
    }

    private void drivebaseReport()
    {
        SmartDashboard.putNumber("Drive Left", tankDriveLeftSpeed);
        SmartDashboard.putNumber("Drive Right" ,tankDriveRightSpeed);
        //SmartDashboard.putNumber("Arcade Drive", arcadeDriveCommand);
        //SmartDashboard.putNumber("Arcade Steer" ,arcadeSteerCommand);
        SmartDashboard.putNumberArray("Drive Turn P-PID", turnLog);
    }

    private void armReport()
    {
        SmartDashboard.putBoolean("Claw Open", clawStatusOpen);
    }
}