package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private PhotonCamera photonCamera; // protect proton camera
    private PhotonTrackedTarget photonTrackedTarget;

    public boolean limelightHasTargets;

    public Vision(){
        photonCamera = new PhotonCamera("687Limelight1");
        photonCamera.setDriverMode(false);
    }

    @Override
    public void periodic() {
        var result = photonCamera.getLatestResult();
        limelightHasTargets = result.hasTargets();
        SmartDashboard.putBoolean("Target Present", limelightHasTargets);

        if (limelightHasTargets) { 
            photonTrackedTarget = result.getBestTarget();
            logToSmartDashboard();
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

    private void logToSmartDashboard() {
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("ID", getFiducialId());
        SmartDashboard.putNumber("Pitch", getPitch());
    }

}