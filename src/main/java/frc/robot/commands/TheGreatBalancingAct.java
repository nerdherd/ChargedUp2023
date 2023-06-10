package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TheGreatBalancingAct extends CommandBase {
    private SwerveDrivetrain swerveDrive;
    private PIDController rollPidController;
    private PIDController pitchPidController;

    private double kPx;
    private double kIx;
    private double kDx;
    private double kPy;
    private double kIy;
    private double kDy;

    // private double towTime = 0;
    // private double period = 0.02;

    /**
     * Construct a new BalancingAct Command
     * @param swerveDrive   Swerve drivetrain to balance
     * @param period        Time intervals for pid controller calculations (default 20ms)
     * @param kP            Constant P for roll and pitch pid controllers
     * @param kI            Constant I for roll and pitch pid controllers
     * @param kD            Constant D for roll and pitch pid controllers
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double period, double kP, double kI, double kD) {
        this.swerveDrive = swerveDrive;

        this.kPx = kP;
        this.kPy = kP;
        this.kIx = kI;
        this.kIy = kI;
        this.kDx = kD;
        this.kDy = kD;

        this.rollPidController = new PIDController(
            kPy, kIy, kDy, period);
        
        this.rollPidController.enableContinuousInput(0, 360);
        
        this.pitchPidController = new PIDController(
            kPx, kIx, kDx, period);
        
        this.pitchPidController.enableContinuousInput(0, 360);
        addRequirements(swerveDrive);

        // this.period = period;
        // SmartDashboard.putNumber("kPBalancing", kP);
        // SmartDashboard.putNumber("kIBalancing", kI);
        // SmartDashboard.putNumber("kDBalancing", kD);
    }

    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double period, double kPX, double kIX, double kDX, double kPY, double kIY, double kDY) {
        this.swerveDrive = swerveDrive;

        this.kPx = kPX;
        this.kIx = kIX;
        this.kDx = kDX;
        this.kPy = kPY;
        this.kIy = kIY;
        this.kDy = kDY;

        this.rollPidController = new PIDController(
            kPy, kIy, kDy, period);
        
        this.rollPidController.enableContinuousInput(0, 360);
        
        this.pitchPidController = new PIDController(
            kPx, kIx, kDx, period);
        
        this.pitchPidController.enableContinuousInput(0, 360);
        addRequirements(swerveDrive);

        // this.period = period;
        // SmartDashboard.putNumber("kPBalancing", kP);
        // SmartDashboard.putNumber("kIBalancing", kI);
        // SmartDashboard.putNumber("kDBalancing", kD);
    }

    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double kPX, double kIX, double kDX, double kPY, double kIY, double kDY) {
        this(swerveDrive, 0.02, kPX, kIX, kDX, kPY, kIY, kDY);
    }
    /**
     * Construct a new BalancingAct Command and assume period is the default (20ms)
     * @param swerveDrive   Swerve drivetrain to balance
     * @param pBalancing    Constant P for roll and pitch pid controllers
     * @param kP            Constant P for roll and pitch pid controllers
     * @param kI            Constant I for roll and pitch pid controllers
     * @param kD            Constant D for roll and pitch pid controllers
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive, double kP, double kI, double kD) {
        this(swerveDrive, 0.02, kP, kI, kD);
    }

    /**
     * Construct a new BalancingAct Command
     * assumes period is the default (20ms)
     * Uses swerve auto constant for roll and pitch pid controllers
     * @param swerveDrive   Swerve drivetrain to balance
     */
    public TheGreatBalancingAct(SwerveDrivetrain swerveDrive) {
        // Default period is 20 ms
        this(swerveDrive, 0.02, 
            SwerveAutoConstants.kPBalancing, 
            SwerveAutoConstants.kIBalancing, 
            SwerveAutoConstants.kDBalancing);
    }

    @Override
    public void initialize() {}

    /**
     * Balances robot by using its pitch and roll as inputs for pid controllers
     * and then sends the values given by the pid controllers to swerve states
     */
    @Override
    public void execute() {
        Rotation3d currentRotation = swerveDrive.getImu().getRotation3d();

        // temporary debugging code
        // kPx = SmartDashboard.getNumber("kPBalancingX", kPx);
        // kIx = SmartDashboard.getNumber("kIBalancingX", kIx);
        // kDx = SmartDashboard.getNumber("kDBalancingX", kDx);
        // kPy = SmartDashboard.getNumber("kPBalancingY", kPy);
        // kIy = SmartDashboard.getNumber("kIBalancingY", kIy);
        // kDy = SmartDashboard.getNumber("kDBalancingY", kDy);
        // pitchPidController.setP(kPx);
        // pitchPidController.setI(kIx);
        // pitchPidController.setD(kDx);
        // rollPidController.setP(kPy);
        // rollPidController.setI(kIy);
        // rollPidController.setD(kDy);

        // Calculate turning speed with PID
        double xSpeed = pitchPidController.calculate(
            currentRotation.getX(), 0
        );
        double ySpeed = -rollPidController.calculate(
            currentRotation.getY(), 0
        );

        // SmartDashboard.putNumber("xSpeed", xSpeed);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
        // SmartDashboard.putNumber("pitch", currentRotation.getY());
        // SmartDashboard.putNumber("roll", currentRotation.getX());
        // SmartDashboard.putNumber("yaw", currentRotation.getZ());

        // Tow the swerve
        // if (currentRotation.getX() < SwerveAutoConstants.kBalancingDeadbandDegrees
        //     && currentRotation.getY() < SwerveAutoConstants.kBalancingDeadbandDegrees) {
        //     towTime += period;
        // } else {
        //     towTime = 0;
        // }

        // if (towTime > SwerveAutoConstants.kBalancingTowPeriod) {
        //     swerveDrive.setModuleStates(SwerveDriveConstants.towModuleStates);
        //     return;
        // }

        // Convert speed into swerve states
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0.0);
        SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Set swerve states
        swerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
