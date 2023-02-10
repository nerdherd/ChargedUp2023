// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Handle input from 687's PS4 Controllers connected to the Driver Station.
 *
 * <p>This class handles PS4 input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 * 
 * <p>This class should be used instead of {@link PS4Controller} due to 687's PS4 controller
 * being recognized as an {@link XboxController} by Windows.
 * 
 * <p>Methods for handling digital input from the L2 and R2 buttons have been disabled, as they
 * are treated as the analog LT and RT triggers on an Xbox controller.
 */
public class BadPS4 extends GenericHID {
    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public BadPS4(int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_PS4Controller, port + 1);
    }

    /** Represents a digital button on an PS4Controller. */
    public enum Button {
        kL1(5),
        kR1(6),
        kL3(9),
        kR3(10),
        kCross(1),
        kCircle(2),
        kSquare(3),
        kTriangle(4),
        kShare(7),
        kOptions(8),
        // Doesn't work
        kPS(13),
        // Doesn't work
        kTouchpad(14);

        public final int value;

        Button(int value) {
        this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
        var name = this.name().substring(1); // Remove leading `k`
        if (name.endsWith("Bumper")) {
            return name;
        }
        return name + "Button";
        }
    }

    /** Represents an axis on an PS4Controller. */
    public enum Axis {
        kLeftX(0),
        kRightX(4),
        kLeftY(1),
        kRightY(5),
        kL2(2),
        kR2(3);

        public final int value;

        Axis(int value) {
        this.value = value;
        }

        /**
         * Get the human-friendly name of the axis, matching the relevant methods. This is done by
         * stripping the leading `k`, and if a trigger axis append `Axis`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the axis.
         */
        @Override
        public String toString() {
        var name = this.name().substring(1); // Remove leading `k`
        if (name.endsWith("Trigger")) {
            return name + "Axis";
        }
        return name;
        }
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return the axis value.
     */
    public double getLeftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return the axis value.
     */
    public double getRightX() {
        return getRawAxis(Axis.kRightX.value);
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return the axis value.
     */
    public double getLeftY() {
        return getRawAxis(Axis.kLeftY.value);
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return the axis value.
     */
    public double getRightY() {
        return getRawAxis(Axis.kRightY.value);
    }

    /**
     * Get the L2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return the axis value.
     */
    public double getL2Axis() {
        return getRawAxis(Axis.kL2.value);
    }

    /**
     * Get the R2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return the axis value.
     */
    public double getR2Axis() {
        return getRawAxis(Axis.kR2.value);
    }

    /**
     * Read the value of the L1 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL1Button() {
        return getRawButton(Button.kL1.value);
    }

    /**
     * Read the value of the R1 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR1Button() {
        return getRawButton(Button.kR1.value);
    }

    /**
     * Whether the L1 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL1ButtonPressed() {
        return getRawButtonPressed(Button.kL1.value);
    }

    /**
     * Whether the R1 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR1ButtonPressed() {
        return getRawButtonPressed(Button.kR1.value);
    }

    /**
     * Whether the L1 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getL1ButtonReleased() {
        return getRawButtonReleased(Button.kL1.value);
    }

    /**
     * Whether the R1 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getR1ButtonReleased() {
        return getRawButtonReleased(Button.kR1.value);
    }

    /**
     * Constructs an event instance around the L1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L1 button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent L1(EventLoop loop) {
        return new BooleanEvent(loop, this::getL1Button);
    }

    /**
     * Constructs an event instance around the R1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R1 button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent R1(EventLoop loop) {
        return new BooleanEvent(loop, this::getR1Button);
    }

    /**
     * Read the value of the L2 button on the controller.
     *
     * @param threshold the threshold before the button registers as true. This value 
     *      should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * 
     * @return The state of the button.
     */
    public boolean getL2Button(double threshold) {
        return getL2Axis() > threshold;
    }

    /**
     * Read the value of the L2 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL2Button() {
        return getL2Button(0.5);
    }

    /**
     * Read the value of the R2 button on the controller.
     *
     * @param threshold the threshold before the button registers as true. This value 
     *      should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * 
     * @return The state of the button.
     */
    public boolean getR2Button(double threshold) {
        return getR2Axis() > threshold;
    }

    /**
     * Read the value of the R2 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR2Button() {
        return getR2Button(0.5);
    }

    //TODO: implement getL2ButtonPressed(), getL2ButtonReleased(), getR2ButtonPressed(), and getR2ButtonReleased()

    /**
     * Constructs an event instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This
     *     value should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    public BooleanEvent L2(double threshold, EventLoop loop) {
        return new BooleanEvent(loop, () -> getL2Axis() > threshold);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    public BooleanEvent L2(EventLoop loop) {
        return L2(0.5, loop);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent} to be true. This
     *     value should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    public BooleanEvent R2(double threshold, EventLoop loop) {
        return new BooleanEvent(loop, () -> getR2Axis() > threshold);
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds the provided
     *     threshold, attached to the given event loop
     */
    public BooleanEvent R2(EventLoop loop) {
        return R2(0.5, loop);
    }

    /**
     * Read the value of the L3 button (pressing the left analog stick) on the controller.
     *
     * @return The state of the button.
     */
    public boolean getL3Button() {
        return getRawButton(Button.kL3.value);
    }

    /**
     * Read the value of the R3 button (pressing the right analog stick) on the controller.
     *
     * @return The state of the button.
     */
    public boolean getR3Button() {
        return getRawButton(Button.kR3.value);
    }

    /**
     * Whether the L3 (left stick) button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getL3ButtonPressed() {
        return getRawButtonPressed(Button.kL3.value);
    }

    /**
     * Whether the R3 (right stick) button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getR3ButtonPressed() {
        return getRawButtonPressed(Button.kR3.value);
    }

    /**
     * Whether the L3 (left stick) button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getL3ButtonReleased() {
        return getRawButtonReleased(Button.kL3.value);
    }

    /**
     * Whether the R3 (right stick) button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getR3ButtonReleased() {
        return getRawButtonReleased(Button.kR3.value);
    }

    /**
     * Constructs an event instance around the L3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L3 button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent L3(EventLoop loop) {
        return new BooleanEvent(loop, this::getL3Button);
    }

    /**
     * Constructs an event instance around the R3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R3 button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent R3(EventLoop loop) {
        return new BooleanEvent(loop, this::getR3Button);
    }

    /**
     * Read the value of the Square button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getSquareButton() {
        return getRawButton(Button.kSquare.value);
    }

    /**
     * Whether the Square button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getSquareButtonPressed() {
        return getRawButtonPressed(Button.kSquare.value);
    }

    /**
     * Whether the Square button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getSquareButtonReleased() {
        return getRawButtonReleased(Button.kSquare.value);
    }

    /**
     * Constructs an event instance around the square button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the square button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent square(EventLoop loop) {
        return new BooleanEvent(loop, this::getSquareButton);
    }

    /**
     * Read the value of the Cross button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getCrossButton() {
        return getRawButton(Button.kCross.value);
    }

    /**
     * Whether the Cross button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getCrossButtonPressed() {
        return getRawButtonPressed(Button.kCross.value);
    }

    /**
     * Whether the Cross button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getCrossButtonReleased() {
        return getRawButtonReleased(Button.kCross.value);
    }

    /**
     * Constructs an event instance around the cross button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the cross button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent cross(EventLoop loop) {
        return new BooleanEvent(loop, this::getCrossButton);
    }

    /**
     * Read the value of the Triangle button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getTriangleButton() {
        return getRawButton(Button.kTriangle.value);
    }

    /**
     * Whether the Triangle button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getTriangleButtonPressed() {
        return getRawButtonPressed(Button.kTriangle.value);
    }

    /**
     * Whether the Triangle button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getTriangleButtonReleased() {
        return getRawButtonReleased(Button.kTriangle.value);
    }

    /**
     * Constructs an event instance around the triangle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the triangle button's digital signal attached to the
     *     given loop.
     */
    public BooleanEvent triangle(EventLoop loop) {
        return new BooleanEvent(loop, this::getTriangleButton);
    }

    /**
     * Read the value of the Circle button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getCircleButton() {
        return getRawButton(Button.kCircle.value);
    }

    /**
     * Whether the Circle button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getCircleButtonPressed() {
        return getRawButtonPressed(Button.kCircle.value);
    }

    /**
     * Whether the Circle button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getCircleButtonReleased() {
        return getRawButtonReleased(Button.kCircle.value);
    }

    /**
     * Constructs an event instance around the circle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the circle button's digital signal attached to the given
     *     loop.
     */
    public BooleanEvent circle(EventLoop loop) {
        return new BooleanEvent(loop, this::getCircleButton);
    }

    /**
     * Read the value of the share button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getShareButton() {
        return getRawButton(Button.kShare.value);
    }

    /**
     * Whether the share button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getShareButtonPressed() {
        return getRawButtonPressed(Button.kShare.value);
    }

    /**
     * Whether the share button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getShareButtonReleased() {
        return getRawButtonReleased(Button.kShare.value);
    }

    /**
     * Constructs an event instance around the share button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the share button's digital signal attached to the given
     *     loop.
     */
    @SuppressWarnings("MethodName")
    public BooleanEvent share(EventLoop loop) {
        return new BooleanEvent(loop, this::getShareButton);
    }

    /**
     * Read the value of the PS button on the controller.
     *
     * @return The state of the button.
     */
    @Deprecated
    public boolean getPSButton() {
        return getRawButton(Button.kPS.value);
    }

    /**
     * Whether the PS button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    @Deprecated
    public boolean getPSButtonPressed() {
        return getRawButtonPressed(Button.kPS.value);
    }

    /**
     * Whether the PS button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    @Deprecated
    public boolean getPSButtonReleased() {
        return getRawButtonReleased(Button.kPS.value);
    }

    /**
     * Constructs an event instance around the PS button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the PS button's digital signal attached to the given
     *     loop.
     */
    @Deprecated
    @SuppressWarnings("MethodName")
    public BooleanEvent PS(EventLoop loop) {
        return new BooleanEvent(loop, this::getPSButton);
    }

    /**
     * Read the value of the options button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getOptionsButton() {
        return getRawButton(Button.kOptions.value);
    }

    /**
     * Whether the options button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getOptionsButtonPressed() {
        return getRawButtonPressed(Button.kOptions.value);
    }

    /**
     * Whether the options button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getOptionsButtonReleased() {
        return getRawButtonReleased(Button.kOptions.value);
    }

    /**
     * Constructs an event instance around the options button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the options button's digital signal attached to the
     *     given loop.
     */
    public BooleanEvent options(EventLoop loop) {
        return new BooleanEvent(loop, this::getOptionsButton);
    }

    /**
     * Read the value of the touchpad on the controller.
     *
     * @return The state of the touchpad.
     */
    @Deprecated
    public boolean getTouchpad() {
        return getRawButton(Button.kTouchpad.value);
    }

    /**
     * Whether the touchpad was pressed since the last check.
     *
     * @return Whether the touchpad was pressed since the last check.
     */
    @Deprecated
    public boolean getTouchpadPressed() {
        return getRawButtonPressed(Button.kTouchpad.value);
    }

    /**
     * Whether the touchpad was released since the last check.
     *
     * @return Whether the touchpad was released since the last check.
     */
    @Deprecated
    public boolean getTouchpadReleased() {
        return getRawButtonReleased(Button.kTouchpad.value);
    }

    /**
     * Constructs an event instance around the touchpad's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the touchpad's digital signal attached to the given
     *     loop.
     */
    @Deprecated
    public BooleanEvent touchpad(EventLoop loop) {
        return new BooleanEvent(loop, this::getTouchpad);
    }
    }
