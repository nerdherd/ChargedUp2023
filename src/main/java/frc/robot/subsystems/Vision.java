package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.util.NerdyMath;

public class Vision extends SubsystemBase implements Reportable{
    
    public enum CAMERA_MODE
    {
        WAIT, // found nothing
        IDLE, // doing nothing, init
        ACTION,// detected one, and approach to it
        ARRIVED // found
    }
    
    public static enum PipelineType {
        // make sure sync with Camera hardware configuration
        NONE(0), CONE(1), CUBE(2),  TAPE(3), ATAG(4);

        private int type;

        private PipelineType(int type) {
            this.type = type;
        }
        public int getType() {
            return type;
        }
    }

    //public Limelight limelightLow;
    //public Limelight limelightHigh;
    public BooleanSupplier cameraHighStatusSupplier;
    public CAMERA_MODE highCameraStatus = CAMERA_MODE.IDLE;

    PhotonCamera cameraLow;// = new PhotonCamera("photonvisionlow");
    public BooleanSupplier cameraLowStatusSupplier;
    CAMERA_MODE lowCameraStatus = CAMERA_MODE.IDLE;

    //private HighLowState state = null;
    private PipelineType pipeline = null;

    public Vision(){
        /*try {
            limelightLow = new Limelight("limelight-kaden");
            limelightLow.setLightState(Limelight.LightMode.OFF);
        } catch (Exception e) {
            System.out.println("low limelight not initialized");
        }
        try {
            limelightHigh = new Limelight("limelight1");
            limelightHigh.setLightState(Limelight.LightMode.OFF);
        } catch (Exception e) {
            System.out.println("high limelight not initialized");
        }*/
        try {
            cameraLow = new PhotonCamera("photonvisionlow");
        } catch (Exception ex) {
            cameraLow = null;
            DriverStation.reportWarning("Error instantiating LOW Camera:  " + ex.getMessage(), true);
        }
        lowCameraStatus = CAMERA_MODE.IDLE;
        cameraLowStatusSupplier = () -> (lowCameraStatus == CAMERA_MODE.ARRIVED);

        highCameraStatus = CAMERA_MODE.IDLE;
        cameraHighStatusSupplier = () -> (highCameraStatus == CAMERA_MODE.ARRIVED);
    }

    @Override
    public void periodic() {

    }


    public void reportToSmartDashboard() {

    }

    /*public CommandBase SwitchLow() {
        return Commands.run(
            () -> SwitchStates(HighLowState.LOW)
        );
    }
    public CommandBase SwitchHigh() {
        return Commands.run(
            () -> SwitchStates(HighLowState.HIGH)
        );
    }*/
    public CommandBase SwitchCone() {
        return Commands.run(
            () -> SwitchPipes(PipelineType.CONE)
        );
    }
    public CommandBase SwitchCube() {
        return Commands.run(
            () -> SwitchPipes(PipelineType.CUBE)
        );
    }
    public CommandBase SwitchTape() {
        return Commands.run(
            () -> SwitchPipes(PipelineType.TAPE)
        );
    }
    public CommandBase SwitchATag() {
        return Commands.run(
            () -> SwitchPipes(PipelineType.ATAG)
        );
    }

    /*private void SwitchStates(HighLowState state) {
        this.state = state;
    }*/

    private void SwitchPipes(PipelineType pipeline) {
        this.pipeline = pipeline;
    }
    /*public Limelight getLimelight(boolean isHigh){
        if(isHigh)
            return limelightHigh;
            else
        return limelightLow; 
    }*/

    /* For Limelight camera
    public void getPPAP(SwerveDrivetrain drivetrain) {
        SmartDashboard.putNumber("tX P", 0.05);
        SmartDashboard.putNumber("area P", 0.24);
        SmartDashboard.putNumber("tX I", 0);
        SmartDashboard.putNumber("area I", 0);
        SmartDashboard.putNumber("tX D", 0);
        SmartDashboard.putNumber("area D", 0);

        limelightLow.setPipeline(1);

        PIDController pidX;
        PIDController pidDistance;

        // Allows for tuning in Dashboard; Get rid of later once everything is tuned
        pidX = new PIDController(SmartDashboard.getNumber("tX P", 0.05), SmartDashboard.getNumber("tX I", 0.05), SmartDashboard.getNumber("tX D", 0.05)); //0.03
        double tolerance = 0.1 * limelightLow.getArea();
        pidX.setTolerance(0.2);
        pidDistance = new PIDController(SmartDashboard.getNumber("area P", 0.05), SmartDashboard.getNumber("area I", 0.05), SmartDashboard.getNumber("area D", 0.05)); //0.05
        pidDistance.setTolerance(tolerance);
        pidX.setTolerance(tolerance);
        double goalArea = 10;

        final double kMaxOutputPercent = 0.6;

        ChassisSpeeds chassisSpeeds;
        double xSpeed;
        double ySpeed;
        if(!limelightLow.hasValidTarget()) {
            xSpeed = 0;
            ySpeed = 0;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            return;    
        }
        

        // double range = 0.628 - 1.71*Math.log(limelight.getArea());
        double objArea = limelightLow.getArea_avg();
        

        
        double calculatedX, calculatedY;



        calculatedX = pidDistance.calculate(objArea, goalArea);
        calculatedY = -pidX.calculate(limelightLow.getXAngle_avg(), 0);
        if(pidDistance.atSetpoint()) {
            xSpeed = 0;
        }
        else {
            xSpeed = calculatedX;
            xSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
        }

        if(pidX.atSetpoint()){
            ySpeed = 0;
        }
        else {
            ySpeed = calculatedY;   // SOMEBODY SWAP THE PIDX and Y NAMES
            ySpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2
        }

        SmartDashboard.putNumber("Vision Tolerance", tolerance);
        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
        SmartDashboard.putNumber("Vision Area", objArea);
        SmartDashboard.putBoolean("Vision has target", limelightLow.hasValidTarget());
        SmartDashboard.putNumber("Limelight x", limelightLow.getXAngle());

        SmartDashboard.putBoolean("Setpoint reached x", pidX.atSetpoint());
        SmartDashboard.putBoolean("Setpoint reached y", pidDistance.atSetpoint());

        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        // drivetrain.setModuleStates(moduleStates);
    }*/
    
    PIDController pidRobotY = new PIDController(0, 0, 0);
    PIDController pidRobotX = new PIDController(0, 0, 0);
    PIDController pidRobotSteer;

    private double lowListRobotX[] = new double[10];
    private int lowListRobotX_idx = 0;
    public double getListAveWithNew_lowRobotX(double newValue)
    {
        lowListRobotX[lowListRobotX_idx] = newValue;
        lowListRobotX_idx ++;
        if(lowListRobotX_idx >= lowListRobotX.length) {
            lowListRobotX_idx = 0;
        }

        double TXSum = 0;
        for(int i = 0; i < lowListRobotX.length; i++) {
            TXSum += lowListRobotX[i];
        }

        return TXSum / lowListRobotX.length;
    }

    
    private double lowListRobotY[] = new double[10];
    private int lowListRobotY_idx = 0;
    public double getListAveWithNew_lowRobotY(double newValue)
    {
        lowListRobotY[lowListRobotY_idx] = newValue;
        lowListRobotY_idx ++;
        if(lowListRobotY_idx >= lowListRobotY.length) {
            lowListRobotY_idx = 0;
        }

        double TXSum = 0;
        for(int i = 0; i < lowListRobotY.length; i++) {
            TXSum += lowListRobotY[i];
        }

        return TXSum / lowListRobotY.length;
    }

    double goalAreaLowCamera = 3;

    public void initObjDetection(boolean isHigh, double targetAreaStop, double headingYaw) {
        if(!isHigh) {
            goalAreaLowCamera = targetAreaStop;
            lowCameraStatus = CAMERA_MODE.IDLE;
            for(int i = 0; i < lowListRobotX.length; i++) {
                lowListRobotX[i] = targetAreaStop;
            }
            if(cameraLow != null)
                cameraLow.setPipelineIndex(0);
        }
        else{

        }
     }

     // PhotonVision for finding CONE on ground
    public void getPPAP(SwerveDrivetrain drivetrain) {
        if(cameraLow == null)
            return;
            
        // TODO !!!!!!!!
        pidRobotY.setP(SmartDashboard.getNumber("tX P", 0.05));
        pidRobotY.setI(SmartDashboard.getNumber("tX I", 0.0));
        pidRobotY.setD(SmartDashboard.getNumber("tX D", 0.05)); //0.03
        pidRobotX.setTolerance(0.2);
        pidRobotX.setP(SmartDashboard.getNumber("area P", 0.05));
        pidRobotX.setI(SmartDashboard.getNumber("area I", 0.0)); 
        pidRobotX.setD(SmartDashboard.getNumber("area D", 0.05)); //0.05
        pidRobotY.setTolerance(0.2);

        ChassisSpeeds chassisSpeeds;
        double xSpeed;
        double ySpeed;

        var result = cameraLow.getLatestResult();

        SmartDashboard.putBoolean("Vision has target", result.hasTargets());

        if(!result.hasTargets()) {
            xSpeed = 0;
            ySpeed = 0;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            lowCameraStatus = CAMERA_MODE.WAIT;
        }
        else {
            double calculatedX = getListAveWithNew_lowRobotX(result.getBestTarget().getArea());
            double calculatedY = getListAveWithNew_lowRobotY(result.getBestTarget().getYaw());
            SmartDashboard.putNumber("Vision robotX avg", calculatedX);
            SmartDashboard.putNumber("Vision robotY avg", calculatedY);

            xSpeed = pidRobotX.calculate(calculatedX, goalAreaLowCamera);
            ySpeed = -pidRobotY.calculate(calculatedY, 0);

            //xSpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*6
            //ySpeed*=SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //*2
            
            // TODO !!!!!!!!
            if(NerdyMath.inRange(xSpeed, -5, 5) &&
            NerdyMath.inRange(ySpeed, -5, 5))
            {
                xSpeed = 0;
                ySpeed = 0;
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                lowCameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                lowCameraStatus = CAMERA_MODE.ACTION;
            }
        }
        
        SmartDashboard.putString("Vision LOW status", lowCameraStatus.toString());

        //SmartDashboard.putNumber("Vision Tolerance", tolerance);
        SmartDashboard.putNumber("Vision X speed", xSpeed);
        SmartDashboard.putNumber("Vision Y speed", ySpeed);
    }


     // for Tape (only with Camera High)
     public void seekTape(SwerveDrivetrain drivetrain) {
     }
    
}