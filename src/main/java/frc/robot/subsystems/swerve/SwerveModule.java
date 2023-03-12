package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Reportable;

public interface SwerveModule extends Reportable {
    public void stop();
    public void resetEncoder();
    public double calibrateEncoder();
    public double resetEncoderToDefault();
    
    public double getDrivePosition();
    public double getTurningPosition();
    public double getDriveVelocity();
    public double getTurningVelocity();
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public void setDesiredState(SwerveModuleState state);
    public void setBreak(boolean breaking);
}
