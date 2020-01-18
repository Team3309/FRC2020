package org.usfirst.frc.team3309;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.usfirst.frc.team3309.commands.shooter.*;

public class OI {

    private enum side {
        left,
        right,
    }

    public static final int LEFT_CLUSTER_1_ID = 1;
    public static final int LEFT_CLUSTER_2_ID = 1;
    public static final int LEFT_CLUSTER_3_ID = 1;
    public static final int LEFT_CLUSTER_4_ID = 1;
    public static final int LEFT_CLUSTER_5_ID = 1;
    public static final int LEFT_CLUSTER_6_ID = 1;
    public static final int RIGHT_CLUSTER_1_ID = 1;
    public static final int RIGHT_CLUSTER_2_ID = 1;
    public static final int RIGHT_CLUSTER_3_ID = 1;
    public static final int RIGHT_CLUSTER_4_ID = 1;
    public static final int RIGHT_CLUSTER_5_ID = 1;
    public static final int RIGHT_CLUSTER_6_ID = 1;
    public static final int UP_BUTTON_ID = 1;
    public static final int DOWN_BUTTON_ID = 1;
    public static final int LEFT_BUTTON_ID = 1;
    public static final int RIGHT_BUTTON_ID = 1;
    public static final int TRIGGER_ID = 1;

    public Joystick leftStick = new Joystick(0);
    public Joystick rightStick = new Joystick(1);
    public XboxController operatorController = new XboxController(2);

    ClusterGroup leftStickLeftCluster = new ClusterGroup(leftStick, side.left);
    ClusterGroup leftStickRightCluster = new ClusterGroup(leftStick, side.right);
    ClusterGroup rightStickLeftCluster = new ClusterGroup(rightStick, side.left);
    ClusterGroup rightStickRightCluster = new ClusterGroup(rightStick, side.right);
    JoystickButton shootingButton = new JoystickButton(leftStick, TRIGGER_ID);

    OI() {
        leftStickLeftCluster.whenActive(new FireAuto());
    }

    private class ClusterGroup extends Trigger {

        Joystick stick;
        side side;

        ClusterGroup(Joystick stick, side side) {
            this.stick = stick;
            this.side = side;
        }

        @Override
        public boolean get() {
            if (side == side.left) {
                return stick.getRawButton(LEFT_CLUSTER_1_ID) ||
                        stick.getRawButton(LEFT_CLUSTER_2_ID) ||
                        stick.getRawButton(LEFT_CLUSTER_3_ID) ||
                        stick.getRawButton(LEFT_CLUSTER_4_ID) ||
                        stick.getRawButton(LEFT_CLUSTER_5_ID) ||
                        stick.getRawButton(LEFT_CLUSTER_6_ID);
            } else {
                return stick.getRawButton(RIGHT_CLUSTER_1_ID) ||
                        stick.getRawButton(RIGHT_CLUSTER_2_ID) ||
                        stick.getRawButton(RIGHT_CLUSTER_3_ID) ||
                        stick.getRawButton(RIGHT_CLUSTER_4_ID) ||
                        stick.getRawButton(RIGHT_CLUSTER_5_ID) ||
                        stick.getRawButton(RIGHT_CLUSTER_6_ID);
            }
        }
    }
}
