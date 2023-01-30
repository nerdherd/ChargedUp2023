package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
 
public abstract class Drivetrain extends SubsystemBase{

    public Drivetrain() {
    }

    public abstract void reportToSmartDashboard();

    public abstract void buildDiffDrive();

    public abstract void zeroDiffDriveSensors();

    public abstract void arcadeDiffDrive(double forward, double turn);
 
    public abstract void tankDrive(double leftInput, double rightInput);

    public abstract void setNeutralCoast();

    public abstract void setPower(double leftPower, double rightPower);

    public abstract void resetEncoders();

    public abstract double getTicks();

    public abstract double meterToTicks(double meterDist);

    public abstract double getApriltagRotation();
   
    public abstract void arcadeDrive(double forwardSpeed, double rotationSpeed);

    public abstract double getApriltagLinear();

    public abstract double getAprilTagAreaLinear();

    public abstract double getHeading();

    public abstract Rotation2d getRotation2d();

    public abstract Rotation3d getRotation3d();

    public abstract void zeroHeading();

    public abstract void resetOdometry(Pose2d pose);

    public abstract void stopModules();

    public abstract Rotation3d getRotation3dRaw();

    public abstract Pose2d getPose();

}
