package org.usfirst.frc.team3309;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.usfirst.frc.team3309.commands.shooter.*;

public class OI {
    Joystick leftStick = new Joystick(0);
    Joystick rightStick = new Joystick(1);
    XboxController operatorController = new XboxController(2);

    JoystickButton shootingButton = new JoystickButton(leftStick, 1);

    OI() {
        shootingButton.whenPressed(new FireAuto());
    }
}
