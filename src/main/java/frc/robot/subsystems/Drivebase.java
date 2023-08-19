package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankConstants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;

public class Drivebase extends SubsystemBase {
    private TalonFX leftMaster;
    private TalonFX leftFollower;
    private TalonFX leftFollower2;
    private TalonFX rightMaster;
    private TalonFX rightFollower;
    private TalonFX rightFollower2;

    private DoubleSolenoid driveShifter;
    private DoubleSolenoid driveShifter2;

    public Drivebase() {
        this.leftMaster = new TalonFX(TankConstants.kLeftMasterID);
        this.leftFollower = new TalonFX(TankConstants.kLeftFollowerID);
        this.leftFollower2 = new TalonFX(TankConstants.kLeftFollower2ID);
        this.rightMaster = new TalonFX(TankConstants.kRightMasterID);
        this.rightFollower = new TalonFX(TankConstants.kRightFollowerID);
        this.rightFollower2 = new TalonFX(TankConstants.kRightFollower2ID);

        this.leftFollower.follow(leftMaster);
        this.leftFollower2.follow(leftMaster);
        this.rightFollower.follow(rightMaster);
        this.rightFollower2.follow(rightMaster);

        this.leftMaster.setInverted(TankConstants.leftMasterInverted);
        this.leftFollower.setInverted(InvertType.FollowMaster);
        this.leftFollower2.setInverted(InvertType.FollowMaster);
        this.rightMaster.setInverted(TankConstants.rightMasterInverted);
        this.rightFollower.setInverted(InvertType.FollowMaster);
        this.rightFollower2.setInverted(InvertType.FollowMaster);

        this.driveShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, TankConstants.kSolenoidForwardID, TankConstants.kSolenoidReverseID);
        this.driveShifter2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, TankConstants.kSolenoidForwardID2, TankConstants.kSolenoidReverseID2);
    }

    public void drive(double leftVelocity, double rightVelocity) {
        this.leftMaster.set(ControlMode.PercentOutput, leftVelocity);
        this.rightMaster.set(ControlMode.PercentOutput, rightVelocity);
    }

    public void toggleShift() {
        this.driveShifter.toggle();
        this.driveShifter2.toggle();
    }

    public void setShift(boolean shift) {
        if (shift) {
            this.driveShifter.set(Value.kForward);
            this.driveShifter2.set(Value.kReverse);
        } else {
            this.driveShifter.set(Value.kReverse);
            this.driveShifter2.set(Value.kForward);
        }
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL)  {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                tab.addNumber("Left Master Current", leftMaster::getStatorCurrent);
                tab.addNumber("Left Follower Current", leftFollower::getStatorCurrent);
                tab.addNumber("Left Follower 2 Current", leftFollower2::getStatorCurrent);
                tab.addNumber("Right Master Current", rightMaster::getStatorCurrent);
                tab.addNumber("Right Follower Current", rightFollower::getStatorCurrent);
                tab.addNumber("Right Follower 2 Current", rightFollower2::getStatorCurrent);
                
            case MINIMAL:
        }
    }

}
