package org.usfirst.frc.team3309;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team3309.commands.shooter.*;

public class OI {

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

    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    XboxController operatorController = new XboxController(2);

    JoystickButton shootingButton = new JoystickButton(leftStick, TRIGGER_ID);

    OI() {
        shootingButton.whenPressed(new FireAuto());
    }
}
