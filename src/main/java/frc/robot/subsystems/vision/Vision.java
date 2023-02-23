package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class Vision extends SubsystemBase implements Reportable{

    public enum OBJECT_TYPE
    {
        CONE,
        CUBE
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

    private Limelight limelightLow;
    private Limelight limelightHigh;
    public BooleanSupplier cameraStatusSupplier;
    private CAMERA_MODE cameraStatus = CAMERA_MODE.IDLE;
    private OBJECT_TYPE currentGameObject;

    public Vision() {
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

        cameraStatus = CAMERA_MODE.IDLE;
        cameraStatusSupplier = () -> (cameraStatus == CAMERA_MODE.ARRIVED);

        cameraStatus = CAMERA_MODE.IDLE;
        cameraStatusSupplier = () -> (cameraStatus == CAMERA_MODE.ARRIVED);

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

    //TODO: Add shuffleboard for vision
    public void initShuffleboard() {

    }

    public void reportToSmartDashboard() {

    }

    
    PIDController pidRobotSteer;

    private double lowListRobotX[] = new double[10];
    private int listRobotX_idx = 0;
    private boolean initDoneListRobotX = false;
    public double getListAveWithNew_lowRobotX(double newValue)
    {
        lowListRobotX[listRobotX_idx] = newValue;
        listRobotX_idx ++;
        if(listRobotX_idx >= lowListRobotX.length) {
            listRobotX_idx = 0;
            initDoneListRobotX = true;
        }

        double TXSum = 0;
        if(initDoneListRobotX) {
            for(int i = 0; i < lowListRobotX.length; i++) {
                TXSum += lowListRobotX[i];
            }

            return TXSum / lowListRobotX.length;
        }
        else {
            for(int i = 0; i < listRobotX_idx; i++) {
                TXSum += lowListRobotX[i];
            }

            return TXSum / listRobotX_idx;
        }
    }

    
    private double lowListRobotY[] = new double[10];
    private int listRobotY_idx = 0;
    private boolean initDoneListRobotY = false;
    public double getListAveWithNew_lowRobotY(double newValue)
    {
        lowListRobotY[listRobotY_idx] = newValue;
        listRobotY_idx ++;
        if(listRobotY_idx >= lowListRobotY.length) {
            listRobotY_idx = 0;
            initDoneListRobotY = true;
        }

        if(initDoneListRobotY) {
            double TXSum = 0;
            for(int i = 0; i < lowListRobotY.length; i++) {
                TXSum += lowListRobotY[i];
            }

            return TXSum / lowListRobotY.length;
        }
        else {
            double TXSum = 0;
            for(int i = 0; i < listRobotY_idx; i++) {
                TXSum += lowListRobotY[i];
            }

            return TXSum / listRobotY_idx;
        }
    }

    double goalAreaLowCamera = 3;
    double goalAreaHighCamera = 1.0; // Change this depending on Object for Detection
    double goalHeadingHighCamera = 0;
    

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
            cameraStatus = CAMERA_MODE.WAIT;
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
                cameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeedRequest, ySpeedRequest, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                cameraStatus = CAMERA_MODE.ACTION;
            }
        }
        
        SmartDashboard.putString("VLowCone status", cameraStatus.toString());

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
            cameraStatus = CAMERA_MODE.WAIT;
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
                cameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                cameraStatus = CAMERA_MODE.ACTION;
            }
        }

        SmartDashboard.putString("VHighTape status", cameraStatus.toString());

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
            cameraStatus = CAMERA_MODE.WAIT;
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
                cameraStatus = CAMERA_MODE.ARRIVED; 
            }
            else{
                chassisSpeeds = new ChassisSpeeds(xSpeedRequest, ySpeedRequest, steerSpeedRequest);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                cameraStatus = CAMERA_MODE.ACTION;
            }
        }

        SmartDashboard.putString("VATag status", cameraStatus.toString());

        SmartDashboard.putNumber("VATag X speed", xSpeedRequest);
        SmartDashboard.putNumber("VATag Y speed", ySpeedRequest);
        SmartDashboard.putNumber("VATag Steer speed", steerSpeedRequest);
    }


    public void initVisionCommands() {
            cameraStatus = CAMERA_MODE.IDLE;
            initDoneListRobotX = false;
            initDoneListRobotY = false;
            listRobotX_idx = 0;
            listRobotY_idx = 0;
    }

    public SequentialCommandGroup VisionPickup() {
        if(currentGameObject == OBJECT_TYPE.CONE) {
            return new SequentialCommandGroup(
                
            );
        }
        else {

        }
    }

    public CommandBase VisionScore() {
        if(currentGameObject == OBJECT_TYPE.CONE) {

        }
        else {

        }
    }
}
