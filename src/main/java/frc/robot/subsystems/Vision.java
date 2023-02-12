package frc.robot.subsystems;

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

public class Vision extends SubsystemBase implements Reportable{
    public Limelight limelightLow;
    public Limelight limelightHigh;

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

    public static enum HighLowState {
        HIGH,
        LOW
    }

    private HighLowState state = null;
    private PipelineType pipeline = null;

    public Vision(){
        try {
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
        }
    
    }

    @Override
    public void periodic() {

    }


    public void reportToSmartDashboard() {

    }

    public CommandBase SwitchLow() {
        return Commands.run(
            () -> SwitchStates(HighLowState.LOW)
        );
    }
    public CommandBase SwitchHigh() {
        return Commands.run(
            () -> SwitchStates(HighLowState.HIGH)
        );
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

    private void SwitchStates(HighLowState state) {
        this.state = state;
    }

    private void SwitchPipes(PipelineType pipeline) {
        this.pipeline = pipeline;
    }
    public Limelight getLimelight(boolean isHigh){
        /*switch (state) {
            case HIGH:
                return limelightHigh;
            case LOW:
                return limelightLow;
            default:
                return null;
        }*/
        if(isHigh)
            return limelightHigh;
            else
        return limelightLow; 
    }

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
    }
    
}