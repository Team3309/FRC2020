package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.CheesyDriveHelper;

public class DriveManual extends CommandBase
{
    private DriveSubsystem Drive;
    private CheesyDriveHelper cheesyDrive = new CheesyDriveHelper();

    public DriveManual(DriveSubsystem drive)
    {
        Drive = drive;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute()
    {
        double throttle = Robot.oi.leftStick.getX();
        double turn = Robot.oi.rightStick.getY();
        boolean quickTurn = Robot.oi.rightStick.getTrigger();

        Drive.SetLeftRight(ControlMode.PercentOutput, cheesyDrive.update(throttle, turn, quickTurn, false));
    }

    public void end() { }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
