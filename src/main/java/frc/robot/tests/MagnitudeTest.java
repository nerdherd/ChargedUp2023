package frc.robot.tests;

import frc.robot.Constants;
import frc.robot.util.BadPS4;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MagnitudeTest {
    BadPS4 controller = new BadPS4(0);
    Joystick joystick = new Joystick(2);
    double xInput = 0;
    double yInput = 0;

    public void init() {
        SmartDashboard.putNumber("X Speed", 0);
        SmartDashboard.putNumber("Y Speed", 0);
    }

    public void periodic() {
        xInput = controller.getLeftY();
        yInput = controller.getLeftX();
        xInput = joystick.getY();
        yInput = joystick.getX();
        // xInput = SmartDashboard.getNumber("X Speed", xInput);
        // yInput = SmartDashboard.getNumber("Y Speed", yInput);
        SmartDashboard.putNumber("Current X Input", xInput);
        SmartDashboard.putNumber("Current Y Input", yInput);
        ChassisSpeeds speeds = new ChassisSpeeds(xInput, yInput, 0);
        SwerveModuleState[] states = Constants.SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SmartDashboard.putNumber("Magnitude", Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2)));
        SmartDashboard.putNumber("Swerve FR Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve FL Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve BL Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve BR Speed", states[3].speedMetersPerSecond);


    }
}
