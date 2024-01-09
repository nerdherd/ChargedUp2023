// package frc.robot.commands.VisionAutos;

// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

// public class FollowVisionPath extends CommandBase {
//     private SwerveAutoBuilder swerveAutoBuilder;
//     private PrimalSunflower sunflower;

//     /**
//      * Construct a new FollowVisionPath command
//      * 
//      * Uses primal sunflower to follow a path using pathplanner trajectory
//      * 
//      * @param autoBuilder Swerve Auto Builder for Path Planner
//      * @param sunflower   Primal Sunflower
//      */
//     public FollowVisionPath(SwerveAutoBuilder autoBuilder, PrimalSunflower sunflower) {
//         this.swerveAutoBuilder = autoBuilder;
//         this.sunflower = sunflower;
//     }

//     @Override
//     public void initialize() {}

//     @Override
//     public void execute() {
//         swerveAutoBuilder.followPathWithEvents(sunflower.toNearestGrid());
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
    
// }