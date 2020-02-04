package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.CheesyDriveHelper;

public class DriveManual extends CommandBase
{
    private DriveSubsystem Drive;
    private Joystick LeftStick;
    private Joystick RightStick;

    private CheesyDriveHelper cheesyDrive = new CheesyDriveHelper();
    Config.RobotModel a = Config.currentRobot;

    public DriveManual(Joystick leftStick, Joystick rightStick, DriveSubsystem drive)
    {
        addRequirements(drive);

        LeftStick = leftStick;
        RightStick = rightStick;
        Drive = drive;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute()
    {
        double throttle = -LeftStick.getY();  // positive getY() is down
        double turn = RightStick.getX();  // positive getX() is to the right
        boolean quickTurn = RightStick.getTrigger();

        Drive.SetLeftRight(ControlMode.PercentOutput, cheesyDrive.update(throttle, turn, quickTurn, false));
    }

    public void end() { }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
