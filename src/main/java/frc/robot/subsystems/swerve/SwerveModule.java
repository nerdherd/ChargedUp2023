package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Reportable;

public interface SwerveModule extends Reportable {
    public void stop();
    public void run();
    public void resetEncoder();
    
    public double getDrivePosition();
    public double getTurningPosition();
    public double getDriveVelocity();
    public double getTurningVelocity();
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public void setDesiredState(SwerveModuleState state);
    public void setDesiredState(SwerveModuleState state, boolean withVelocityControl);
    public void setTurnOffset(double offset);
    public double getTurnOffset();
    public void toggleVelocityControl(boolean velocityControlOn);
    public void setBreak(boolean breaking);
}
