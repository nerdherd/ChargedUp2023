package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

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

    public enum VisionTask
    {
        CONE_ON_GROUND,
        TAPE_ON_POLE,
        ATAG_GRID
    }
    
    public enum CAMERA_MODE
    {
        WAIT, // found nothing
        IDLE, // doing nothing, init
        ACTION,// detected one, and approach to it
        ARRIVED // found
    }
    
    public static enum PipelineType {
        NONE(0), CONE(1), CUBE(2),  TAPE(3), ATAG(4);

        private int type;

        private PipelineType(int type) {
            this.type = type;
        }
        public int getType() {
            return type;
        }
    }

    public Limelight limelightLow;
    public Limelight limelightHigh;
    public BooleanSupplier cameraHighStatusSupplier;
    public CAMERA_MODE highCameraStatus = CAMERA_MODE.IDLE;

    public BooleanSupplier cameraLowStatusSupplier;
    CAMERA_MODE lowCameraStatus = CAMERA_MODE.IDLE;

    private PipelineType pipeline = null;

    public Vision(){
        try {
            limelightHigh = new Limelight("limelight-high");
            limelightHigh.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightHigh = null;
            DriverStation.reportWarning("Error instantiating high camera:  " + ex.getMessage(), true);
        }

        try {
            limelightLow = new Limelight("limelight-low");
            limelightLow.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelightLow = null;
            DriverStation.reportWarning("Error instantiating low camera:  " + ex.getMessage(), true);
        }

        highCameraStatus = CAMERA_MODE.IDLE;
        cameraHighStatusSupplier = () -> (highCameraStatus == CAMERA_MODE.ARRIVED);

        lowCameraStatus = CAMERA_MODE.IDLE;
        cameraLowStatusSupplier = () -> (lowCameraStatus == CAMERA_MODE.ARRIVED);

        //low cone
        SmartDashboard.putNumber("VLowCone X P", 5);
        SmartDashboard.putNumber("VLowCone Y P", 4);
        SmartDashboard.putNumber("VLowCone X I", 0);
        SmartDashboard.putNumber("VLowCone Y I", 0);
        SmartDashboard.putNumber("VLowCone X D", 0);
        SmartDashboard.putNumber("VLowCone Y D", 0);

        SmartDashboard.putNumber("VLowCone Target X", goalAreaLowCamera); // watching


        //atag grid
        SmartDashboard.putNumber("VATag Target X", goalAreaHighCamera);
        SmartDashboard.putNumber("VATag Target Steer", goalHeadingHighCamera);

        
        //tape on pole
        SmartDashboard.putNumber("VHighTape X P", 7);
        SmartDashboard.putNumber("VHighTape Y P", 0.1);
        SmartDashboard.putNumber("VHighTape X I", 0);
        SmartDashboard.putNumber("VHighTape Y I", 0);
        SmartDashboard.putNumber("VHighTape X D", 0);
        SmartDashboard.putNumber("VHighTape Y D", 0);

        SmartDashboard.putNumber("VHighTape Steer P", 0);
        SmartDashboard.putNumber("VHighTape Steer I", 0);
        SmartDashboard.putNumber("VHighTape Steer D", 0);

        SmartDashboard.putNumber("VHighTape Target X", goalAreaHighCamera);
        SmartDashboard.putNumber("VHighTape Target Steer", goalHeadingHighCamera);
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
    double goalAreaHighCamera = 1.0; // Change this depending on Object for Detection
    double goalHeadingHighCamera = 0;
    public void initObjDetection(VisionTask task, double targetAreaStop, double headingYaw) {
        if(task == VisionTask.CONE_ON_GROUND) { // cone on ground
            goalAreaLowCamera = targetAreaStop;
            lowCameraStatus = CAMERA_MODE.IDLE;
            initDoneLowListRobotX = false;
            initDoneLowListRobotY = false;
            lowListRobotX_idx = 0;
            lowListRobotY_idx = 0;
            if(limelightLow != null)
                limelightLow.setPipeline(1);;
            
            
        }
        else if (task == VisionTask.ATAG_GRID){ // april tag; Right now its on APRIL TAG
            goalAreaHighCamera = targetAreaStop;
            highCameraStatus = CAMERA_MODE.IDLE;

            if(limelightHigh != null ){
                limelightHigh.reinitBuffer();
                limelightHigh.setPipeline(4);
            }
            
            goalHeadingHighCamera = headingYaw;
        } 
        else if(task == VisionTask.TAPE_ON_POLE) {
            goalAreaHighCamera = targetAreaStop;
            highCameraStatus = CAMERA_MODE.IDLE;
            limelightHigh.turnLightOn();
            if(limelightHigh != null ){
                limelightHigh.reinitBuffer();
                limelightHigh.setPipeline(3);
            }
            
            goalHeadingHighCamera = headingYaw;
        }
        else{}

        xSpeedRequest = 0;
        ySpeedRequest = 0;
        steerSpeedRequest = 0;
     }

    double xSpeedRequest = 0;
    double ySpeedRequest = 0;
    double steerSpeedRequest = 0;

    // Limelight for finding CONE on ground
    PIDController pidRobotY_lowCone = new PIDController(0, 0, 0);
    PIDController pidRobotX_lowCone = new PIDController(0, 0, 0);
    public void getPPAP(SwerveDrivetrain drivetrain) {
        if(limelightLow == null)
            return;
            
        pidRobotY_lowCone.setP(SmartDashboard.getNumber("VLowCone Y P", 0.05));
        pidRobotY_lowCone.setI(SmartDashboard.getNumber("VLowCone Y I", 0.0));
        pidRobotY_lowCone.setD(SmartDashboard.getNumber("VLowCone Y D", 0.05)); //0.03
        pidRobotY_lowCone.setTolerance(0.2);

        pidRobotX_lowCone.setP(SmartDashboard.getNumber("VLowCone X P", 0.05));
        pidRobotX_lowCone.setI(SmartDashboard.getNumber("VLowCone X I", 0.0)); 
        pidRobotX_lowCone.setD(SmartDashboard.getNumber("VLowCone X D", 0.05)); //0.05
        pidRobotX_lowCone.setTolerance(0.2);

        ChassisSpeeds chassisSpeeds;

        SmartDashboard.putBoolean("VLowCone has target", limelightLow.hasValidTarget());

        if(!limelightLow.hasValidTarget()) {
            xSpeedRequest = 0;
            ySpeedRequest = 0;
            chassisSpeeds = new ChassisSpeeds(xSpeedRequest, ySpeedRequest, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            lowCameraStatus = CAMERA_MODE.WAIT;
        }
        else {
            double calculatedX = getListAveWithNew_lowRobotX(limelightLow.getArea_avg());// watching pv-area
            double calculatedY = getListAveWithNew_lowRobotY(limelightLow.getXAngle_avg());// watching pv-yaw
            SmartDashboard.putNumber("VLowCone robotX avg", calculatedX);// watching
            SmartDashboard.putNumber("VLowCone robotY avg", calculatedY);// watching

            xSpeedRequest = pidRobotX_lowCone.calculate(calculatedX, goalAreaLowCamera);
            ySpeedRequest = -pidRobotY_lowCone.calculate(calculatedY, 0);
            
            if(NerdyMath.inRange(xSpeedRequest, -.1, .1) &&
            NerdyMath.inRange(ySpeedRequest, -.1, .1))
            {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                lowCameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeedRequest, ySpeedRequest, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                lowCameraStatus = CAMERA_MODE.ACTION;
            }
        }
        
        SmartDashboard.putString("VLowCone status", lowCameraStatus.toString());

        SmartDashboard.putNumber("VLowCone X speed", xSpeedRequest);// watching
        SmartDashboard.putNumber("VLowCone Y speed", ySpeedRequest);// watching
    }


     // for Tape (only with Camera High)
    PIDController pidRobotY_highTape = new PIDController(0, 0, 0);
    PIDController pidRobotX_highTape = new PIDController(0, 0, 0);
    PIDController pidRobotSteer_highTape = new PIDController(0, 0, 0);
    public void seekTape(SwerveDrivetrain drivetrain) {
        if(limelightHigh == null)
            return;
        
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
    
    // for April Tag (only with Camera High)
    PIDController pidRobotY_ATag = new PIDController(0, 0, 0);
    PIDController pidRobotX_ATag = new PIDController(0, 0, 0);
    PIDController pidRobotSteer_ATag = new PIDController(0, 0, 0);
    public void seekATag(SwerveDrivetrain drivetrain) {
        if(limelightHigh == null)
            return;
        
        // Allows for tuning in Dashboard; Get rid of later once everything is tuned
        // TODO !!!!!!!!
        pidRobotY_ATag.setP(SmartDashboard.getNumber("VATag Y P", 0.5));
        pidRobotY_ATag.setI(SmartDashboard.getNumber("VATag Y I", 0.0));
        pidRobotY_ATag.setD(SmartDashboard.getNumber("VATag Y D", 0.5));
        pidRobotY_ATag.setTolerance(0.5);

        pidRobotX_ATag.setP(SmartDashboard.getNumber("VATag X P", 0.5));
        pidRobotX_ATag.setI(SmartDashboard.getNumber("VATag X I", 0.0)); 
        pidRobotX_ATag.setD(SmartDashboard.getNumber("VATag X D", 1.5)); 
        pidRobotX_ATag.setTolerance(0.5);

        pidRobotSteer_ATag.setP(SmartDashboard.getNumber("VATag Steer P", 0));
        pidRobotSteer_ATag.setI(SmartDashboard.getNumber("VATag Steer I", 0)); 
        pidRobotSteer_ATag.setD(SmartDashboard.getNumber("VATag Steer D", 0)); 

        ChassisSpeeds chassisSpeeds;

        SmartDashboard.putBoolean("VATag has target", limelightHigh.hasValidTarget());
        

        if(!limelightHigh.hasValidTarget()) {
            //xSpeedRequest = 0;
            //ySpeedRequest = 0;
            //steerSpeedRequest = 0;
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            highCameraStatus = CAMERA_MODE.WAIT;
        }
        else {        
            double calculatedX = limelightHigh.getArea_avg();
            double calculatedY = limelightHigh.getXAngle_avg();
            double calculatedSteer = drivetrain.getImu().getHeading();
            SmartDashboard.putNumber("VATag robotX avg", calculatedX);
            SmartDashboard.putNumber("VATag robotY avg", calculatedY);
            SmartDashboard.putNumber("VATag robotSteer avg", calculatedSteer);

            //xSpeedRequest = pidRobotX_ATag.calculate(calculatedX, goalAreaHighCamera);
            // Calculate the error terms for the PID controllers
            double errorX =  calculatedX - goalAreaHighCamera;
            double errorY = calculatedY - 0;
            double errorSteer = calculatedSteer - goalHeadingHighCamera;
            
            /* // if needed
            // Apply additional logic to handle off-center and angled targets
            if (Math.abs(errorY) > 5) {
                // If the target is off-center, add an additional correction term to the x PID controller
                errorX -= 0.1 * errorY;
            }
            if (Math.abs(errorSteer) > 10) {
                // If the target is at an angle, add an additional correction term to the y PID controller
                errorY -= 0.1 * errorSteer;
            }*/


            xSpeedRequest = pidRobotX_ATag.calculate(errorX);
            ySpeedRequest = -pidRobotY_ATag.calculate(errorY);
            steerSpeedRequest = pidRobotSteer_ATag.calculate(errorSteer);

            if(!NerdyMath.inRange(errorSteer, -2, 2))
            {
                steerSpeedRequest += 0.2;
            }

            
            // TODO !!!!!!!!
            if(NerdyMath.inRange(xSpeedRequest, -0.05, 0.05) &&
            NerdyMath.inRange(ySpeedRequest, -0.05, 0.05)&&
            NerdyMath.inRange(steerSpeedRequest, -0.05, 0.05))
            {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                highCameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeedRequest, ySpeedRequest, steerSpeedRequest);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                highCameraStatus = CAMERA_MODE.ACTION;
            }
        }

        SmartDashboard.putString("VATag status", highCameraStatus.toString());

        SmartDashboard.putNumber("VATag X speed", xSpeedRequest);
        SmartDashboard.putNumber("VATag Y speed", ySpeedRequest);
        SmartDashboard.putNumber("VATag Steer speed", steerSpeedRequest);
    }
}
