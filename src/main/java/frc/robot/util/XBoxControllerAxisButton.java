package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;

public class XBoxControllerAxisButton extends InternalButton {

    private final XboxController controller;
    private final XboxController.Axis axis;
    private final double threshold;
    private boolean previousState = false;

    public XBoxControllerAxisButton(XboxController controller, XboxController.Axis axis, double threshold) {
        this.controller = controller;
        this.axis = axis;
        this.threshold = threshold;
    }

    public boolean get() {
        if (axis == XboxController.Axis.kLeftTrigger) {
            boolean newState = controller.getTriggerAxis(GenericHID.Hand.kLeft) >= threshold;
            if (!(newState || previousState && !(newState && previousState))) {
                if (previousState) {
                    previousState = false;
                } else {
                    previousState = true;
                }
                return true;
            }
            return false;
        } else if (axis == XboxController.Axis.kRightTrigger) {
            boolean newState = controller.getTriggerAxis(GenericHID.Hand.kRight) >= threshold;
            if (!(newState || previousState && !(newState && previousState))) {
                if (previousState) {
                    previousState = false;
                } else {
                    previousState = true;
                }
                return true;
            }
            return false;
        } else if (axis == XboxController.Axis.kLeftX) {
            return controller.getX(GenericHID.Hand.kLeft) >= threshold;
        } else if (axis == XboxController.Axis.kRightX) {
            return controller.getX(GenericHID.Hand.kRight) >= threshold;
        } else if (axis == XboxController.Axis.kLeftY) {
            return controller.getY(GenericHID.Hand.kLeft) >= threshold;
        } else if (axis == XboxController.Axis.kRightY) {
            return controller.getY(GenericHID.Hand.kRight) >= threshold;
        }
        return false;
    }

}
