package frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveSimpleTest extends CommandBase {

    private DriveSubsystem Drive;
    private double Speed;

    public DriveSimpleTest(double speed, DriveSubsystem driveSubsystem)
    {
        addRequirements(driveSubsystem);

        Drive = driveSubsystem;
        Speed = speed;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        Drive.SetLeftRight(ControlMode.PercentOutput, Speed, Speed);
    }

    @Override
    public void end(boolean interrupted) { }
}
