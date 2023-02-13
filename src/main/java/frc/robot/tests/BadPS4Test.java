package frc.robot.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.util.CommandBadPS4;
import frc.robot.util.BadPS4;

public class BadPS4Test {
    BadPS4 badPs4;
    CommandBadPS4 commandPS4;
    POVButton upButton;
    POVButton rightButton;
    POVButton downButton;
    POVButton leftButton;


    public void initialize() {
        badPs4 = new BadPS4(0);
        upButton = new POVButton(badPs4, 0);
        downButton = new POVButton(badPs4, 180);
        rightButton = new POVButton(badPs4, 90);
        leftButton = new POVButton(badPs4, 270);
    }


    public void testBadPS4Periodic() {
        SmartDashboard.putBoolean("Square", badPs4.getSquareButton());
        SmartDashboard.putBoolean("Triangle", badPs4.getTriangleButton());
        SmartDashboard.putBoolean("Circle", badPs4.getCircleButton());
        SmartDashboard.putBoolean("Cross", badPs4.getCrossButton());
        SmartDashboard.putNumber("Right X", badPs4.getRightX());
        SmartDashboard.putNumber("Right Y", badPs4.getRightY());
        SmartDashboard.putNumber("Left X", badPs4.getLeftX());
        SmartDashboard.putNumber("Left Y", badPs4.getLeftY());
        SmartDashboard.putNumber("L2", badPs4.getL2Axis());
        SmartDashboard.putNumber("R2", badPs4.getR2Axis());
        SmartDashboard.putBoolean("L1", badPs4.getL1Button());
        SmartDashboard.putBoolean("R1", badPs4.getR1Button());
        SmartDashboard.putBoolean("L3", badPs4.getL3Button());
        SmartDashboard.putBoolean("R3", badPs4.getR3Button());
        SmartDashboard.putBoolean("L2 Pressed", badPs4.getL2Button());
        SmartDashboard.putBoolean("R2 Pressed", badPs4.getR2Button());
        SmartDashboard.putBoolean("PS", badPs4.getPSButton());
        SmartDashboard.putBoolean("Share", badPs4.getShareButton());
        SmartDashboard.putBoolean("Options", badPs4.getOptionsButton());
        // SmartDashboard.putBoolean("Touchpad", badPs4.getTouchpad());
    }

    public void commandPS4TestInit() {
        commandPS4 = new CommandBadPS4(0);

        // new JoystickButton(badPs4, BadPS4.Button.kL1.value).onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("L1 Pressed", true)))
        // .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("L1 Pressed", false)));

        commandPS4.L1().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("L1 Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("L1 Pressed", false)));
        
        commandPS4.R1().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("R1 Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("R1 Pressed", false)));
        
        commandPS4.L2().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("L2 Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("L2 Pressed", false)));
        
        commandPS4.R2().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("R2 Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("R2 Pressed", false)));
        
        commandPS4.L3().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("L3 Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("L3 Pressed", false)));
        
        commandPS4.R3().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("R3 Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("R3 Pressed", false)));

        commandPS4.PS().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("PS Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("PS Pressed", false)));
        
        commandPS4.share().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Share Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Share Pressed", false)));
        
        commandPS4.options().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Options Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Options Pressed", false)));
        
        commandPS4.triangle().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Triangle Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Triangle Pressed", false)));
        
        commandPS4.square().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Square Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Square Pressed", false)));
        
        commandPS4.circle().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Circle Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Circle Pressed", false)));   

        commandPS4.cross().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Cross Pressed", true)))
            .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Cross Pressed", false)));   
        
        upButton.onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Up Pressed", true)))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Up Pressed", false))); 
        
        downButton.onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Down Pressed", true)))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Down Pressed", false))); 

        leftButton.onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Left Pressed", true)))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Left Pressed", false))); 

        rightButton.onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("Right Pressed", true)))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putBoolean("Right Pressed", false))); 
    }
}
