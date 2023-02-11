package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ApproachCombined;
import frc.robot.commands.DriveToTarget;

public class Vision extends SubsystemBase implements Reportable {
    private Limelight limelightLow = new Limelight("limelight");
    private Limelight limelightHigh = new Limelight("limelight1");
    private SwerveDrivetrain drivetrain;

    private Limelight currentLimelight;
    private HighLowState currentHighLowState;
    private PIDController currentPIDX;
    private PIDController currentPIDDistance;
    private PIDController PIDAngular;
    private PipelineType currentPipeline;


    public static enum PipelineType {
        CONE,
        CUBE,
        TAPE,
        ATAG
    }

    public static enum HighLowState {
        HIGH,
        LOW
    }

    private HighLowState state = null;
    private PipelineType pipeline = null;

    public Vision(SwerveDrivetrain drivetrain){
        try {
            limelightLow = new Limelight("limelight");
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

        this.drivetrain = drivetrain;

        // Set default limelight to low
        currentLimelight = limelightLow;

        PIDAngular = new PIDController(0, 0, 0); // Someone tune this please
    }

    public void reportToSmartDashboard() {
        SmartDashboard.putBoolean("Vision target found", currentLimelight.hasValidTarget());
        SmartDashboard.putString("Current Limelight", currentLimelight.toString());
        SmartDashboard.putString("Current Pipeline", currentPipeline.toString());
    }

    // TODO: Add controller rumble or some kind of feedback when setpoint is reached
    public CommandBase ChaseTarget(PipelineType pipeline) {
        SwitchPipes(pipeline);

        if ((pipeline == PipelineType.CONE || pipeline == PipelineType.CUBE) 
                && currentHighLowState == HighLowState.HIGH) { // Substation pickup
            return new ApproachCombined(drivetrain, 180, VisionConstants.kSubstationPickupArea, currentLimelight, currentPIDX, currentPIDDistance);

        } else if ((pipeline == PipelineType.CONE || pipeline == PipelineType.CUBE) 
                && currentHighLowState == HighLowState.LOW) { // Ground pickup
            return new DriveToTarget(drivetrain, currentLimelight, VisionConstants.kFloorPickupArea, currentPIDX, currentPIDDistance);

        } else if (pipeline == PipelineType.ATAG) { // April tag align
            return new ApproachCombined(drivetrain, 0, VisionConstants.kTagAlignArea, currentLimelight, currentPIDX, currentPIDDistance);

        } else if (pipeline == PipelineType.TAPE) { // Tape align
            return new ApproachCombined(drivetrain, 0, VisionConstants.kTapeAlignArea, currentLimelight, currentPIDX, currentPIDDistance);
        
        } else {
            return null;
        }

    }

    public CommandBase SwitchLow() {
        return Commands.runOnce(
            () -> SwitchLimelight(HighLowState.LOW)
        );
    }
    public CommandBase SwitchHigh() {
        return Commands.runOnce(
            () -> SwitchLimelight(HighLowState.HIGH)
        );
    }

    private void SwitchLimelight(HighLowState highLowState) {
        currentHighLowState = highLowState;
        switch(highLowState) {
            case HIGH:
                currentLimelight = limelightHigh;
            case LOW:
                currentLimelight = limelightLow;
        }
    }

    // TODO: Set tolerances here, right now they're being set in DriveToTarget()
    private void SwitchPipes(PipelineType pipeline) {
        currentPipeline = pipeline;

        switch(pipeline) {
            case CONE:
                currentLimelight.setPipeline(1);
                currentPIDX = new PIDController(0.045, 0, 0.01); // Values from tuning at DaVinci 2/4/23
                currentPIDDistance = new PIDController(0.2, 0.01, 0.02);
                break;

            case CUBE:
                currentLimelight.setPipeline(2);
                currentPIDX = new PIDController(0.045, 0, 0.01);
                currentPIDDistance = new PIDController(0.2, 0.01, 0.02);
                break;

            case TAPE:
                currentLimelight.setPipeline(3);
                currentPIDX = new PIDController(0.045, 0, 0.01);
                currentPIDDistance = new PIDController(0.2, 0.01, 0.02);
                break;

            case ATAG:
                currentLimelight.setPipeline(4);
                currentPIDX = new PIDController(0.045, 0, 0.01);
                currentPIDDistance = new PIDController(0.2, 0.01, 0.02);
                break;
        }
    }
}