package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
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
    public Limelight limelightHigh;
    public BooleanSupplier cameraHighStatusSupplier;
    public CAMERA_MODE highCameraStatus = CAMERA_MODE.IDLE;

    PhotonCamera photonVisionLow;// = new PhotonCamera("photonvisionlow");
    public BooleanSupplier cameraLowStatusSupplier;
    CAMERA_MODE lowCameraStatus = CAMERA_MODE.IDLE;

    //private HighLowState state = null;
    private PipelineType pipeline = null;

    public Vision(){
        try {
            limelightHigh = new Limelight("limelight-low");
            limelightHigh.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightHigh = null;
            DriverStation.reportWarning("Error instantiating High Camera:  " + ex.getMessage(), true);
        }

        highCameraStatus = CAMERA_MODE.IDLE;
        cameraHighStatusSupplier = () -> (highCameraStatus == CAMERA_MODE.ARRIVED);

        try {
            photonVisionLow = new PhotonCamera("photonvisionlow");
        } catch (Exception ex) {
            photonVisionLow = null;
            DriverStation.reportWarning("Error instantiating LOW Camera:  " + ex.getMessage(), true);
        }

        lowCameraStatus = CAMERA_MODE.IDLE;
        cameraLowStatusSupplier = () -> (lowCameraStatus == CAMERA_MODE.ARRIVED);
    }

    @Override
    public void periodic() {

    }


    public void reportToSmartDashboard() {

    }

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

    private void SwitchPipes(PipelineType pipeline) {
        this.pipeline = pipeline;
    }
    
    PIDController pidRobotSteer;

    private double lowListRobotX[] = new double[10];
    private int lowListRobotX_idx = 0;
    private boolean initDoneLowListRobotX = false;
    public double getListAveWithNew_lowRobotX(double newValue)
    {
        lowListRobotX[lowListRobotX_idx] = newValue;
        lowListRobotX_idx ++;
        if(lowListRobotX_idx >= lowListRobotX.length) {
            lowListRobotX_idx = 0;
            initDoneLowListRobotX = true;
        }

        double TXSum = 0;
        if(initDoneLowListRobotX) {
            for(int i = 0; i < lowListRobotX.length; i++) {
                TXSum += lowListRobotX[i];
            }

            return TXSum / lowListRobotX.length;
        }
        else {
            for(int i = 0; i < lowListRobotX_idx; i++) {
                TXSum += lowListRobotX[i];
            }

            return TXSum / lowListRobotX_idx;
        }
    }

    
    private double lowListRobotY[] = new double[10];
    private int lowListRobotY_idx = 0;
    private boolean initDoneLowListRobotY = false;
    public double getListAveWithNew_lowRobotY(double newValue)
    {
        lowListRobotY[lowListRobotY_idx] = newValue;
        lowListRobotY_idx ++;
        if(lowListRobotY_idx >= lowListRobotY.length) {
            lowListRobotY_idx = 0;
            initDoneLowListRobotY = true;
        }

        if(initDoneLowListRobotY) {
            double TXSum = 0;
            for(int i = 0; i < lowListRobotY.length; i++) {
                TXSum += lowListRobotY[i];
            }

            return TXSum / lowListRobotY.length;
        }
        else {
            double TXSum = 0;
            for(int i = 0; i < lowListRobotY_idx; i++) {
                TXSum += lowListRobotY[i];
            }

            return TXSum / lowListRobotY_idx;
        }
    }

    double goalAreaLowCamera = 3;
    double goalAreaHighCamera = 0.3;
    public void initObjDetection(boolean isHigh, double targetAreaStop, double headingYaw) {
        if(!isHigh) { // cone on ground
            goalAreaLowCamera = targetAreaStop;
            lowCameraStatus = CAMERA_MODE.IDLE;
            initDoneLowListRobotX = false;
            initDoneLowListRobotY = false;
            lowListRobotX_idx = 0;
            lowListRobotY_idx = 0;
            if(photonVisionLow != null)
                photonVisionLow.setPipelineIndex(0);
            
            SmartDashboard.putNumber("VLowCone X P", 0.25);
            SmartDashboard.putNumber("VLowCone Y P", 0.05);
            SmartDashboard.putNumber("VLowCone X I", 0);
            SmartDashboard.putNumber("VLowCone Y I", 0);
            SmartDashboard.putNumber("VLowCone X D", 0);
            SmartDashboard.putNumber("VLowCone Y D", 0);
        }
        else{ // tape
            goalAreaHighCamera = targetAreaStop;
            highCameraStatus = CAMERA_MODE.IDLE;
            if(limelightHigh != null ){
                limelightHigh.reinitBuffer();
                limelightHigh.setPipeline(3);
            }
            // limelight yaw = headingYaw; // have to use IMU to get the job done. TODO

            SmartDashboard.putNumber("VHighTape X P", 0.05);
            SmartDashboard.putNumber("VHighTape Y P", 0.24);
            SmartDashboard.putNumber("VHighTape X I", 0.005);
            SmartDashboard.putNumber("VHighTape Y I", 0);
            SmartDashboard.putNumber("VHighTape X D", 0);
            SmartDashboard.putNumber("VHighTape Y D", 0);
        }
     }

    

    // Limelight for finding CONE on ground
    PIDController pidRobotY_lowCone = new PIDController(0, 0, 0);
    PIDController pidRobotX_lowCone = new PIDController(0, 0, 0);
    public void getPPAP(SwerveDrivetrain drivetrain) {

        if (limelightHigh == null) 
            return;
            
        // TODO !!!!!!!!
        pidRobotY_lowCone.setP(SmartDashboard.getNumber("VLowCone Y P", 0.05)); // 0.24
        pidRobotY_lowCone.setI(SmartDashboard.getNumber("VLowCone Y I", 0.0));
        pidRobotY_lowCone.setD(SmartDashboard.getNumber("VLowCone Y D", 0.05)); //0.03 // 0
        pidRobotY_lowCone.setTolerance(0.2);

        pidRobotX_lowCone.setP(SmartDashboard.getNumber("VLowCone X P", 0.05));
        pidRobotX_lowCone.setI(SmartDashboard.getNumber("VLowCone X I", 0.0)); 
        pidRobotX_lowCone.setD(SmartDashboard.getNumber("VLowCone X D", 0.05)); //0.05 // 0
        pidRobotX_lowCone.setTolerance(0.2);

        ChassisSpeeds chassisSpeeds;
        double xSpeed;
        double ySpeed;

        SmartDashboard.putBoolean("VLowCone has target", limelightHigh.hasValidTarget());

        if(!limelightHigh.hasValidTarget()) {
            xSpeed = 0;
            ySpeed = 0;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            lowCameraStatus = CAMERA_MODE.WAIT;
        }
        else {
            double calculatedX = getListAveWithNew_lowRobotX(limelightHigh.getArea());
            double calculatedY = getListAveWithNew_lowRobotY(limelightHigh.getXAngle());
            SmartDashboard.putNumber("VLowCone robotX avg", calculatedX);
            SmartDashboard.putNumber("VLowCone robotY avg", calculatedY);

            xSpeed = pidRobotX_lowCone.calculate(calculatedX, goalAreaLowCamera);
            ySpeed = -pidRobotY_lowCone.calculate(calculatedY, 0);
            
            // TODO !!!!!!!!
            if(NerdyMath.inRange(xSpeed, -.1, .1) &&
            NerdyMath.inRange(ySpeed, -.1, .1))
            {
                xSpeed = 0;
                ySpeed = 0;
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                // lowCameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                // lowCameraStatus = CAMERA_MODE.ACTION;
            }
        }
        
        SmartDashboard.putString("VLowCone status", lowCameraStatus.toString());

        //SmartDashboard.putNumber("Vision Tolerance", tolerance);
        SmartDashboard.putNumber("VLowCone X speed", xSpeed);
        SmartDashboard.putNumber("VLowCone Y speed", ySpeed);
    }


     // for Tape (only with Camera High)
    PIDController pidRobotY_highTape = new PIDController(0, 0, 0);
    PIDController pidRobotX_highTape = new PIDController(0, 0, 0);
    PIDController pidRobotSteer_highTape = new PIDController(0, 0, 0);
    public void seekTape(SwerveDrivetrain drivetrain) {
        if(limelightHigh == null)
            return;
        
        // Allows for tuning in Dashboard; Get rid of later once everything is tuned
        // TODO !!!!!!!!
        pidRobotY_highTape.setP(SmartDashboard.getNumber("VHighTape Y P", 0.5));
        pidRobotY_highTape.setI(SmartDashboard.getNumber("VHighTape Y I", 0.0));
        pidRobotY_highTape.setD(SmartDashboard.getNumber("VHighTape Y D", 0.5)); //0.03
        pidRobotY_highTape.setTolerance(0.1);

        pidRobotX_highTape.setP(SmartDashboard.getNumber("VHighTape X P", 0.5));
        pidRobotX_highTape.setI(SmartDashboard.getNumber("VHighTape X I", 0.0)); 
        pidRobotX_highTape.setD(SmartDashboard.getNumber("VHighTape X D", 0.5)); //0.05
        pidRobotX_highTape.setTolerance(0.1);

        ChassisSpeeds chassisSpeeds;
        double xSpeed = 0;
        double ySpeed = 0;
        double steerSpeed = 0;

        if(!limelightHigh.hasValidTarget()) {
            xSpeed = 0;
            ySpeed = 0;
            steerSpeed = 0;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            highCameraStatus = CAMERA_MODE.WAIT;
        }
        else {        
            double calculatedX = limelightHigh.getArea_avg();
            double calculatedY = limelightHigh.getXAngle_avg();
            SmartDashboard.putNumber("VHighTape robotX avg", calculatedX);
            SmartDashboard.putNumber("VHighTape robotY avg", calculatedY);

            xSpeed = pidRobotX_highTape.calculate(calculatedX, goalAreaHighCamera);
            ySpeed = -pidRobotY_highTape.calculate(calculatedY, 0);
            
            // TODO !!!!!!!!
            if(NerdyMath.inRange(xSpeed, -5, 5) &&
            NerdyMath.inRange(ySpeed, -5, 5))
            {
                xSpeed = 0;
                ySpeed = 0;
                steerSpeed = 0;
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                highCameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                highCameraStatus = CAMERA_MODE.ACTION;
            }
        }

        SmartDashboard.putString("VHighTape status", highCameraStatus.toString());

        SmartDashboard.putNumber("VHighTape X speed", xSpeed);
        SmartDashboard.putNumber("VHighTape Y speed", ySpeed);
        SmartDashboard.putNumber("VHighTape Steer speed", steerSpeed);
    }
    
}