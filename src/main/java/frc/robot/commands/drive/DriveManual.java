package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.CheesyDriveHelper;

public class DriveManual extends CommandBase {
    private DriveSubsystem drive;
    private Joystick leftStick;
    private Joystick rightStick;

    private CheesyDriveHelper cheesyDrive = new CheesyDriveHelper();
    Config.RobotModel a = Config.currentRobot;

    public DriveManual(Joystick leftStick, Joystick rightStick, DriveSubsystem drive)
    {
        addRequirements(drive);

        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.drive = drive;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute()
    {
        double throttle = -leftStick.getY();  // positive getY() is down
        double turn = rightStick.getX();  // positive getX() is to the right
        boolean quickTurn = rightStick.getTrigger();

        drive.setLeftRight(ControlMode.PercentOutput, cheesyDrive.update(throttle, turn, quickTurn, false));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
