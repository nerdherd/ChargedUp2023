package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase implements Reportable{
    public Limelight limelightLow = null;
    public Limelight limelightHigh = null;

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

    public Vision(){


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
}