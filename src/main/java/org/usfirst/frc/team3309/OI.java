package org.usfirst.frc.team3309;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.usfirst.frc.team3309.commands.aimer.*;
import org.usfirst.frc.team3309.commands.balancer.*;
import org.usfirst.frc.team3309.commands.climber.*;
import org.usfirst.frc.team3309.commands.ctrlpanelturner.*;
import org.usfirst.frc.team3309.commands.drive.*;
import org.usfirst.frc.team3309.commands.pcindexer.*;
import org.usfirst.frc.team3309.commands.pcintake.*;
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
    public Timer controlTimer = new Timer();

    ClusterGroup leftStickLeftCluster = new ClusterGroup(leftStick, side.left);
    ClusterGroup leftStickRightCluster = new ClusterGroup(leftStick, side.right);
    ClusterGroup rightStickLeftCluster = new ClusterGroup(rightStick, side.left);
    ClusterGroup rightStickRightCluster = new ClusterGroup(rightStick, side.right);
    JoystickButton shootingButton = new JoystickButton(leftStick, TRIGGER_ID);

    public boolean triggerPressed;
    public boolean B_ButtonHadBeenPressed;
    public boolean X_ButtonPressed;
    public boolean Y_ButtonPressed;
    public boolean northDPadPressed;
    public boolean southDPadPressed;
    public boolean westDPadPressed;
    public boolean eastDPadPressed;

    public OI() {

        leftStickLeftCluster.whenActive(new FireAuto());


        if (shootingButton.get()) {
            triggerPressed = shootingButton.get();
            new FireManual();
        }

        //Deploys or retracts turner based on a toggle. Works off of
        if (!B_ButtonHadBeenPressed && operatorController.getBButton()) {
            B_ButtonHadBeenPressed = true;
            new DeployTurner();
        } else if (B_ButtonHadBeenPressed && !operatorController.getBButton()) {
            B_ButtonHadBeenPressed = false;
            new RetractTurner();
        }

        //Tries to obtain Rotation Control if the turner is deployed and the "Y" button is pressed.
        if (B_ButtonHadBeenPressed && operatorController.getYButton()) {
            new RotatePanel();
        }
        //Tries to obtain Position Control if the turner is deployed and the "X" button is pressed.
        if (B_ButtonHadBeenPressed && operatorController.getBButton()) {
            new RotateToColor();
        }

        if (operatorController.getAButton()) {
            new LoadBall();
        }

    }

    private class ClusterGroup extends Trigger {

        Joystick stick;
        OI.side side;

        ClusterGroup(Joystick stick, OI.side side) {
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
