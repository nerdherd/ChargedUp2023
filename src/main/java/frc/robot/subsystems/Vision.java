package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase implements Reportable{
    private PhotonCamera photonCamera = null; // protect proton camera
    private PhotonTrackedTarget photonTrackedTarget;

    public boolean limelightHasTargets;

    public Vision(){
        try { // attempt to instantiate the NavX2. If it throws an exception, catch it and
            // report it.
            photonCamera = new PhotonCamera("687Limelight1");
            photonCamera.setDriverMode(false);
      } catch (RuntimeException ex) {
            limelightHasTargets = false;
          DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
      }
        

    }

    @Override
    public void periodic() {
        var result = photonCamera.getLatestResult();
        limelightHasTargets = result.hasTargets();
        SmartDashboard.putBoolean("Target Present", limelightHasTargets);

        if (limelightHasTargets) { 
            photonTrackedTarget = result.getBestTarget();
            reportToSmartDashboard();
        } 
    }

    public double getPitch() {
        return photonTrackedTarget.getPitch();
    }

    public double getYaw() {
        return photonTrackedTarget.getYaw();
    }

    public int getFiducialId() {
        return photonTrackedTarget.getFiducialId();
    }

    public double getArea(){
        return photonTrackedTarget.getArea();
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("ID", getFiducialId());
        SmartDashboard.putNumber("Pitch", getPitch());
    }

}