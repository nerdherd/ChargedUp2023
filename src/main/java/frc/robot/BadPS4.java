package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;

/**
 * Handle input from the off-brand PS4 controllers that 687 uses.
 * 
 * On these controllers, the button bindings are switched:
 * <p>
 * PS4 --> BadPS4
 * <p>
 * Circle --> Square
 * <p>
 * Square --> Cross
 * <p>
 * Cross --> Circle
 * <p>
 * Triangle --> Triangle
 */
public class BadPS4 extends PS4Controller {
    /** Construct a BadPS4 controller. */
    public BadPS4(int port) {
        super(port);
    }
    
    /** Represents a digital button on an off-brand PS4 Controller. */
    public enum Button {
        kSquare(3), // Originally Circle
        kCross(1), // Originally Square
        kCircle(2), // Originally Cross
        kTriangle(4),
        kL1(5),
        kR1(6),
        kL2(7),
        kR2(8),
        kShare(9),
        kOptions(10),
        kL3(11),
        kR3(12),
        kPS(13),
        kTouchpad(14);
        public final int value;
        Button(int index) {
            this.value = index;
        }
        /**
         * Get the human-friendly name of the button, matching the relevant methods.
         * This is done by
         * stripping the leading `k`, and if not the touchpad append `Button`.
         *
         * <p>
         * Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (this == kTouchpad) {
                return name;
            }
            return name + "Button";
        }
    }
}