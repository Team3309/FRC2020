package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorInterface
{
    // -- Driver
    public Joystick DriverLeft = new Joystick(0);
    public Joystick DriverRight = new Joystick(1);

    // -- Operator
    public XboxController OperatorController = new XboxController(2);


}
